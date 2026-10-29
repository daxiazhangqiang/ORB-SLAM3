#ifndef DACHENG_MAP_QUALITY_MONITOR_H
#define DACHENG_MAP_QUALITY_MONITOR_H
#include "MapQuality.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace dacheng {
// 质量门限为待现场校准的工程默认值，不作为导航精度保证
class MapQualityMonitor {
public:
    explicit MapQualityMonitor(const std::string& prefix, bool validation=false)
        : prefix_(prefix), validation_(validation), csv_(prefix + ".csv") {
        csv_ << "time,state,map_id,active_map_id,inliers,stable_matches,grid_cells,stable_sample,points,coverage,stable,good\n";
    }
    // 离线只读检查不产生实机回访结论
    void SetInspection(bool inspection) { inspection_ = inspection; }
    // 每帧累计真实跟踪时间，短时丢失不因低频质量采样而漏记
    void ObserveState(double t, int state) {
        if (start_ < 0) start_ = t;
        // 帧间丢失或采集空档立即撤销连续稳定证据，不能被低频采样漏过
        if (state != 2 || (last_frame_ >= 0 && t-last_frame_ > 1.0))
            stable_since_ = -1;
        if (last_frame_ >= 0 && t-last_frame_ > 1.0) window_.clear();
        if (last_frame_ >= 0 && first_ok_ >= 0) {
            const double dt = std::max(0.0, t-last_frame_);
            elapsed_ += dt;
            if (last_state_ == 2 && dt <= 1.0) ok_time_ += dt;
            if (last_state_ != 2 || dt > 1.0) {
                lost_run_ += dt;
            } else {
                // 只有达到2秒的丢失才计作一次“较长丢失”，短暂丢失不累计事件
                if (lost_run_ >= 2.0) {
                    ++long_loss_events_;
                    long_loss_total_ += lost_run_;
                }
                lost_run_ = 0;
            }
            max_lost_ = std::max(max_lost_, lost_run_);
        }
        if (state == 2 && first_ok_ < 0) first_ok_ = t;
        last_frame_ = t; last_state_ = state;
    }
    // 用保存主图轨迹范围验证实际移动，避免静止时观察远处特征刷满锚点
    void SetRouteExtent(double extent) { required_extent_=extent; }
    void ObservePosition(double x, double y, double z) {
        const std::array<double,3> p{{x,y,z}};
        if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)) return;
        if(!position_seen_) { minimum_=p; maximum_=p; position_seen_=true; }
        for(int i=0;i<3;++i) { minimum_[i]=std::min(minimum_[i],p[i]); maximum_[i]=std::max(maximum_[i],p[i]); }
    }
    // 计算本次验证移动范围与保存轨迹范围的比例，不使用任意地图单位冒充米
    double MotionCoverage() const {
        if(!position_seen_ || required_extent_<=1e-6) return 0;
        double sum=0; for(int i=0;i<3;++i) sum+=(maximum_[i]-minimum_[i])*(maximum_[i]-minimum_[i]);
        return std::sqrt(sum)/required_extent_;
    }
    // 低频采样更新路线覆盖，地图身份或大回环改变时撤销旧在线评估
    bool Update(const ORB_SLAM3::MapQualitySample& sample, double t, bool inventory) {
        if (sample.map_id != sample_.map_id || sample.revision != sample_.revision) {
            // 融合/回环会改变地图ID或版本，但关键帧ID保留；覆盖历史应继续有效
            window_.clear(); stable_since_ = -1;
            // 全程时长和丢失历史不随换图或回环清零
        }
        const int stable_points = inventory ? sample.stable_points : sample_.stable_points;
        const int tested = inventory ? sample.tested_points : sample_.tested_points;
        sample_ = sample; sample_.stable_points = stable_points; sample_.tested_points = tested;
        // 定位连续性是抽查门限；网格分布只作为诊断，不作为失败条件
        const bool tracking_stable = sample.state == 2 && sample.active_map_id == sample.map_id &&
            sample.inliers >= 80 && sample.stable_matches >= 40;
        const bool good = tracking_stable && sample.grid_cells >= 8;
        if (tracking_stable) {
            for (auto id : sample.visible_anchors) {
                auto& item = seen_[id];
                if (item.first == 0) item.second = t;
                if (item.first == 0 || t-item.second >= 1.0) { ++item.first; item.second = t; }
            }
            if (stable_since_ < 0) stable_since_ = t;
        } else stable_since_ = -1;
        ++samples_; if (tracking_stable) ++stable_samples_; if (good) ++good_samples_;
        inliers_.push_back(sample.state == 2 ? sample.inliers : 0);
        window_.push_back({t, tracking_stable});
        while (!window_.empty() && t-window_.front().first > 30) window_.pop_front();
        csv_ << t << ',' << sample.state << ',' << sample.map_id << ',' << sample.active_map_id << ','
             << sample.inliers << ','
             << sample.stable_matches << ',' << sample.grid_cells << ',' << stable_points << ','
             << sample.points << ',' << Coverage() << ',' << tracking_stable << ',' << good << '\n'; csv_.flush();
        const bool candidate = Candidate(t);
        std::string status;
        if(inspection_)
            status="只读检查：仅统计已保存主图的几何质量，不代表实机回访通过";
        else if(validation_)
            status = candidate ? "回访指标达到参考值；停车后输入 d 进入尺度计算" :
                "回访指标有提示项；停车后输入 d 进入尺度计算，输入 s 可直接跳过回访";
        else if(sample.state != 2)
            status="跟踪未恢复：先停稳或回到已建图区域，等待跟踪状态OK";
        else if(sample.active_map_id != sample.map_id)
            status="当前激活 map_" + std::to_string(sample.active_map_id) +
                   "，不是主图 map_" + std::to_string(sample.map_id) +
                   "；新增点会进入激活子图，先回到主图或等待融合";
        else if(!sample_.SubmapsWithinLimit())
            status="继续建图/建议补扫：存在未融合子图点数超过主图20%，回到重叠区域或检查回环";
        else if(candidate)
            status = validation_ ? "抽查指标达标，请停车并确认已完成一圈后输入 d" :
                "可进入回访验证：主图跟踪稳定，请先停车，输入 v 保存后进入回访抽查";
        else
            status = sample.keyframes < 30 ? "继续建图：平移增加可靠观测" :
                     "继续建图/建议补扫：增加不同位置和朝向的可靠观测";
        if (status != last_hint_ || t-last_print_ >= 5) {
            std::cout << "[地图质量] " << status << "\n  主图=" << sample.map_id;
            if (!inspection_)
                std::cout << " 激活=" << sample.active_map_id << " 状态=" << sample.state;
            std::cout << " 关键帧=" << sample.keyframes
                << " 稳定点抽样下界=" << stable_points << '/' << tested << "（总点=" << sample.points
                << "）内点=" << sample.inliers << " 稳定匹配=" << sample.stable_matches
                << " 网格=" << sample.grid_cells << "/12";
            if (!inspection_)
                std::cout << " 路线锚点覆盖(参考)=" << int(Coverage()*100)
                    << "% 回访移动范围(参考)=" << MotionCoverage()
                    << " 历史最大连续丢失=" << max_lost_ << "s";
            std::cout << std::endl;
            if (!inspection_ && sample.state == 2 && sample.active_map_id == sample.map_id) {
                if (status != last_hint_) {
                    if (validation_)
                        std::cout << "[地图质量] 抽查参考值（不阻止尺度计算）：首次定位<=15s 抽查>=60s 跟踪比例>=90%"
                                  << " 最长丢失<=5s 2s以上丢失<=2次 稳定采样>=85% 内点P10>=80\n"
                                  << "           全部回访指标仅作提示，不作为进入尺度计算的判据\n";
                    else
                        std::cout << "[地图质量] 建图门限：稳定点>=300 内点>=80 稳定匹配>=40；"
                                  << "每个未融合子图点数<=主图20%；冻结回访阶段再抽查定位连续性\n";
                }
                const std::string missing = Missing(t);
                std::cout << "[地图质量] ";
                if (!missing.empty())
                    std::cout << (validation_ ? "参考提示：" : "还差：") << missing;
                else if (candidate)
                    std::cout << (validation_ ? "当前采样已满足抽查门限" :
                                  "建图门限已满足，可进入回访抽查");
                else
                    std::cout << "还差：保持主图连续合格观测";
                std::cout << std::endl;
            } else if (!inspection_) {
                std::cout << "[地图质量] 还差：先恢复到主图并保持跟踪OK，再评估抽查连续性"
                          << std::endl;
            }
            last_hint_=status; last_print_=t;
        }
        last_t_=t;
        return tracking_stable;
    }
    // 统计最终主图锚点，旧锚点不计入覆盖率
    double Coverage() const {
        if (sample_.route_anchors.empty()) return 0;
        int covered=0;
        for (auto id : sample_.route_anchors) {
            auto it=seen_.find(id);
            if (it!=seen_.end() && it->second.first>=2) ++covered;
        }
        return double(covered)/sample_.route_anchors.size();
    }
    // 在线只给候选提示，冻结地图验证额外要求时长、首次定位及全程稳定性
    bool Candidate(double t) const {
        int recent_total=0,recent_stable=0;
        for (const auto& entry: window_) {
            if (t-entry.first <= 10.0) { ++recent_total; recent_stable += entry.second; }
        }
        const double recent_ratio = recent_total ? double(recent_stable)/recent_total : 0;
        // 建图阶段要求主图稳定且未融合子图没有超过主图点数20%
        const bool base = sample_.keyframes>=30 && sample_.stable_points>=300 &&
            sample_.route_anchors.size()>=3 && WindowAge(t)>=10 && recent_total>=5 && recent_ratio>=.8 &&
            stable_since_>=0 && t-stable_since_>=5 && sample_.SubmapsWithinLimit();
        if (!validation_) return base;
        // 抽查只看定位连续性：覆盖率、移动范围和网格分布都只作参考
        const double stable_ratio = double(stable_samples_)/std::max(1,samples_);
        const double long_loss_ratio = LongLossTotal()/std::max(1.0,elapsed_);
        return base && last_state_==2 && elapsed_>=60 && first_ok_>=0 && first_ok_-start_<=15 &&
            ok_time_/std::max(1.0,elapsed_)>=.90 && max_lost_<=5.0 &&
            LongLossEvents()<=2 && long_loss_ratio<=.10 &&
            samples_>=30 && stable_ratio>=.85 && P10()>=80;
    }
    // 分开记录回访完成状态和参考指标结论，不作为尺度采集门禁
    void Save(bool confirmed=false) {
        std::ofstream out(prefix_ + ".json");
        out << "{\n\"schema\":1,\"threshold_profile\":\"engineering_v1_uncalibrated\","
            << "\"mode\":\"" << (inspection_?"inspect":validation_?"frozen_validation":"mapping") << "\","
            << "\"verdict\":\"" << (confirmed && validation_ ? "COMPLETED" : "INCOMPLETE") << "\","
            << "\"metrics_advisory_only\":true,\"metrics_verdict\":\""
            << (Candidate(last_t_) ? "PASS" : "WARN") << "\","
            << "\"metrics_hint\":\"" << Missing(last_t_) << "\","
            << "\"user_route_confirmed\":" << (confirmed?"true":"false") << ','
            << "\"map_id\":" << sample_.map_id << ",\"keyframes\":" << sample_.keyframes
            << ",\"points\":" << sample_.points << ",\"stable_sample_lower_bound\":" << sample_.stable_points
            << ",\"tested_points\":" << sample_.tested_points << ",\"coverage\":" << Coverage()
            << ",\"motion_extent_ratio\":" << MotionCoverage()
            << ",\"duration_s\":" << elapsed_ << ",\"tracking_ratio\":" << ok_time_/std::max(1.0,elapsed_)
            << ",\"first_localization_s\":" << (first_ok_<0?-1:first_ok_-start_)
            << ",\"max_lost_s\":" << max_lost_ << ",\"inliers_p10\":" << P10()
            << ",\"stable_sample_ratio\":" << double(stable_samples_)/std::max(1,samples_)
            << ",\"good_sample_ratio\":" << double(good_samples_)/std::max(1,samples_)
            << ",\"long_loss_events\":" << LongLossEvents()
            << ",\"long_loss_total_s\":" << LongLossTotal()
            << ",\"unmerged_submap_point_limit\":0.2"
            << ",\"unmerged_submap_limit_ok\":" << (sample_.SubmapsWithinLimit()?"true":"false")
            << ",\"unmerged_submaps\":[";
        bool submap_comma=false;
        for(const auto& submap: sample_.other_submaps) {
            if(submap_comma) out << ',';
            const double ratio = sample_.points>0 ? double(submap.second)/sample_.points : 0.0;
            out << "{\"map_id\":" << submap.first << ",\"points\":" << submap.second
                << ",\"primary_ratio\":" << std::fixed << std::setprecision(6) << ratio << '}';
            submap_comma=true;
        }
        out << "],\"uncovered_anchor_ids\":[";
        bool comma=false;
        for(auto id: sample_.route_anchors) {
            auto it=seen_.find(id);
            if(it==seen_.end() || it->second.first<2) { out << (comma?",":"") << id; comma=true; }
        }
        out << "],\"scope\":\"tracking_stability_brief_loss_and_unmerged_submap_point_limit\"}\n";
    }
private:
    // 换图或采集空档后重新积累完整观察时长，与两秒采样周期兼容
    double WindowAge(double t) const {
        return window_.empty() ? 0.0 : std::max(0.0, t-window_.front().first);
    }
    // 列出当前采样相对门限还缺什么，避免只看到“建议补扫”
    std::string Missing(double t) const {
        std::ostringstream out;
        auto num=[](double value) {
            std::ostringstream text; text << std::fixed << std::setprecision(1) << value;
            return text.str();
        };
        auto add=[&](const std::string& name,double have,double need,const std::string& unit) {
            if (have + 1e-9 >= need) return;
            if (out.tellp() > 0) out << "，";
            out << name << ' ' << num(have) << '/' << num(need) << unit
                << "（差" << num(need-have) << unit << "）";
        };
        add("关键帧", sample_.keyframes, 30, "");
        add("稳定点", sample_.stable_points, 300, "");
        add("内点", sample_.inliers, 80, "");
        add("稳定匹配", sample_.stable_matches, 40, "");
        int recent_total=0,recent_stable=0;
        for (const auto& entry: window_) if (t-entry.first <= 10.0) {
            ++recent_total; recent_stable += entry.second;
        }
        add("路线锚点数", sample_.route_anchors.size(), 3, "");
        add("观察窗口", WindowAge(t), 10, "s");
        add("最近10秒采样数", recent_total, 5, "");
        add("最近10秒稳定率", recent_total ? 100.0*recent_stable/recent_total : 0, 80, "%");
        if (stable_since_ < 0 || t-stable_since_ < 5)
            add("连续稳定", stable_since_<0 ? 0 : t-stable_since_, 5, "s");
        for(const auto& submap: sample_.other_submaps) {
            if(static_cast<long long>(submap.second)*5 <= sample_.points) continue;
            if(out.tellp()>0) out << "，";
            const double ratio = sample_.points>0 ? 100.0*submap.second/sample_.points : 0.0;
            std::ostringstream ratio_text;
            ratio_text << std::fixed << std::setprecision(2) << ratio;
            const long long allowed = sample_.points/5;
            out << "map_" << submap.first << "点数 " << submap.second << '/' << sample_.points
                << "（" << ratio_text.str() << "% > 20.0%，超出 " << submap.second-allowed << " 点）";
        }
        if (validation_) {
            add("抽查时长", elapsed_, 60, "s");
            add("跟踪比例", 100.0*ok_time_/std::max(1.0,elapsed_), 90, "%");
            add("稳定采样率", 100.0*stable_samples_/std::max(1,samples_), 85, "%");
            add("内点P10", P10(), 80, "");
            add("采样数", samples_, 30, "");
            if (first_ok_ < 0)
                out << (out.tellp()>0 ? "，" : "") << "尚未首次定位";
            else if (first_ok_-start_ > 15)
                out << (out.tellp()>0 ? "，" : "") << "首次定位 " << num(first_ok_-start_)
                    << "/15.0s（超出" << num(first_ok_-start_-15) << "s）";
            if (max_lost_ > 5.0)
                out << (out.tellp()>0 ? "，" : "") << "最长丢失 " << num(max_lost_) << "/5.0s"
                    << "（超出" << num(max_lost_-5.0) << "s）";
            if (LongLossEvents() > 2)
                out << (out.tellp()>0 ? "，" : "") << "2s以上丢失 " << LongLossEvents()
                    << "/2次（超出" << LongLossEvents()-2 << "次）";
            if (LongLossTotal()/std::max(1.0,elapsed_) > .10)
                out << (out.tellp()>0 ? "，" : "") << "较长丢失累计 "
                    << num(LongLossTotal()) << "s/10%时长（超出"
                    << num(LongLossTotal()-0.10*std::max(1.0,elapsed_)) << "s）";
        }
        return out.str();
    }
    // 统计超过2秒的丢失事件；正在进行的丢失也计入
    int LongLossEvents() const {
        return long_loss_events_ + (lost_run_>=2.0 ? 1 : 0);
    }
    // 统计超过2秒的丢失总时长；正在进行的丢失也计入
    double LongLossTotal() const {
        return long_loss_total_ + (lost_run_>=2.0 ? lost_run_ : 0.0);
    }
    // 用低分位数表达弱区段，避免平均值掩盖匹配低谷
    int P10() const { auto v=inliers_; if(v.empty()) return 0; std::sort(v.begin(),v.end()); return v[(v.size()-1)/10]; }
    std::string prefix_,last_hint_;
    bool validation_,inspection_=false;
    std::ofstream csv_;
    ORB_SLAM3::MapQualitySample sample_;
    std::map<unsigned long,std::pair<int,double>> seen_;
    std::deque<std::pair<double,bool>> window_;
    std::vector<int> inliers_;
    int samples_=0,stable_samples_=0,good_samples_=0,long_loss_events_=0,last_state_=-1;
    bool position_seen_=false;
    double required_extent_=0;
    std::array<double,3> minimum_{{0,0,0}}, maximum_{{0,0,0}};
    double start_=-1,first_ok_=-1,last_frame_=-1,elapsed_=0,ok_time_=0,lost_run_=0,max_lost_=0;
    double long_loss_total_=0,stable_since_=-1,last_print_=-1,last_t_=0;
};
}
#endif
