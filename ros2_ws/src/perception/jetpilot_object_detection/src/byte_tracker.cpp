#include "jetpilot_object_detection/byte_tracker.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace jetpilot_object_detection {
namespace {
float iou(const Detection & a, const Detection & b) {
  const float intersection = std::max(0.F, std::min(a.x_max,b.x_max)-std::max(a.x_min,b.x_min)) *
    std::max(0.F, std::min(a.y_max,b.y_max)-std::max(a.y_min,b.y_min));
  const float area = (a.x_max-a.x_min)*(a.y_max-a.y_min)+(b.x_max-b.x_min)*(b.y_max-b.y_min)-intersection;
  return area>0 ? intersection/area : 0;
}
// Rectangular Hungarian assignment; per-row dummy columns allow unmatched tracks.
std::vector<int> assign(const std::vector<std::vector<double>> & cost) {
  const int n=static_cast<int>(cost.size());
  if(!n) return {};
  const int m=static_cast<int>(cost[0].size());
  std::vector<double> u(n+1),v(m+1);
  std::vector<int> p(m+1),way(m+1);
  for(int i=1;i<=n;++i) {
    p[0]=i; int j0=0;
    std::vector<double> minv(m+1,std::numeric_limits<double>::infinity());
    std::vector<bool> used(m+1,false);
    do {
      used[j0]=true; const int i0=p[j0]; int j1=0;
      double delta=std::numeric_limits<double>::infinity();
      for(int j=1;j<=m;++j) if(!used[j]) {
        const double cur=cost[i0-1][j-1]-u[i0]-v[j];
        if(cur<minv[j]) {minv[j]=cur;way[j]=j0;}
        if(minv[j]<delta) {delta=minv[j];j1=j;}
      }
      for(int j=0;j<=m;++j) {
        if(used[j]) {u[p[j]]+=delta;v[j]-=delta;} else minv[j]-=delta;
      }
      j0=j1;
    } while(p[j0]);
    do {const int j1=way[j0];p[j0]=p[j1];j0=j1;} while(j0);
  }
  std::vector<int> result(n,-1);
  for(int j=1;j<=m;++j) if(p[j]) result[p[j]-1]=j-1;
  return result;
}
}
ByteTracker::ByteTracker(TrackerConfig config):config_(config) {
  if(!std::isfinite(config.low_score)||!std::isfinite(config.high_score)||!std::isfinite(config.new_score)||
     !std::isfinite(config.match_iou)||!std::isfinite(config.lost_seconds)||
     config.low_score<0||config.low_score>config.high_score||config.high_score>config.new_score||
     config.new_score>1||config.match_iou<=0||config.match_iou>1||config.lost_seconds<=0||
     !config.min_hits||!config.max_tracks) throw std::invalid_argument("invalid tracker thresholds/limits");
}
std::vector<TrackMatch> ByteTracker::update(const std::vector<Detection> & detections,
  double timestamp, const std::string & frame) {
  if(!std::isfinite(timestamp)||frame.empty()) return {};
  if(initialized_ && (timestamp<last_time_ || frame!=frame_)) tracks_.clear();
  else if(initialized_ && timestamp==last_time_) return {};
  initialized_=true;last_time_=timestamp;frame_=frame;
  tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[&](const Track & t){
    return timestamp-t.seen>config_.lost_seconds;
  }),tracks_.end());
  std::vector<std::size_t> high,low;
  for(std::size_t i=0;i<detections.size();++i) {
    const auto & d=detections[i];
    if(!std::isfinite(d.score)||d.score>1||!std::isfinite(d.x_min)||!std::isfinite(d.y_min)||
       !std::isfinite(d.x_max)||!std::isfinite(d.y_max)||d.x_max<=d.x_min||d.y_max<=d.y_min) continue;
    if(d.score>=config_.high_score) high.push_back(i);
    else if(d.score>=config_.low_score) low.push_back(i);
  }
  std::vector<bool> matched(tracks_.size(),false),used(detections.size(),false);
  std::vector<TrackMatch> output;
  auto stage=[&](const std::vector<std::size_t> & candidates, bool low_stage) {
    std::vector<std::size_t> eligible;
    for(std::size_t i=0;i<tracks_.size();++i)
      if(!matched[i] && (!low_stage || (!tracks_[i].missed && tracks_[i].hits>=config_.min_hits))) eligible.push_back(i);
    std::vector<std::vector<double>> cost(eligible.size(),std::vector<double>(candidates.size()+eligible.size(),1.0));
    for(std::size_t r=0;r<eligible.size();++r) {
      const auto & t=tracks_[eligible[r]];auto predicted=t.box;
      const float dt=static_cast<float>(timestamp-t.seen);
      predicted.x_min+=t.velocity[0]*dt;predicted.y_min+=t.velocity[1]*dt;
      predicted.x_max+=t.velocity[2]*dt;predicted.y_max+=t.velocity[3]*dt;
      for(std::size_t c=0;c<candidates.size();++c) {
        const auto & d=detections[candidates[c]];const float overlap=iou(predicted,d);
        cost[r][c]=(t.box.class_id==d.class_id && overlap>=config_.match_iou) ? 1.0-overlap : 1e6;
      }
    }
    const auto assignment=assign(cost);
    for(std::size_t r=0;r<eligible.size();++r) {
      const int col=assignment[r];
      if(col<0||static_cast<std::size_t>(col)>=candidates.size()||cost[r][col]>=1.0) continue;
      const auto index=candidates[col];auto & t=tracks_[eligible[r]];const auto & d=detections[index];
      const double dt=timestamp-t.seen;
      if(dt>1e-6) {
        const std::array<float,4> delta{d.x_min-t.box.x_min,d.y_min-t.box.y_min,d.x_max-t.box.x_max,d.y_max-t.box.y_max};
        for(int k=0;k<4;++k) t.velocity[k]=0.5F*t.velocity[k]+0.5F*delta[k]/static_cast<float>(dt);
      }
      t.box=d;t.seen=timestamp;++t.hits;t.missed=false;
      matched[eligible[r]]=true;used[index]=true;
      if(t.hits>=config_.min_hits) output.push_back({index,t.id});
    }
  };
  stage(high,false);stage(low,true);
  for(std::size_t i=0;i<tracks_.size();++i) if(!matched[i]) tracks_[i].missed=true;
  // Unconfirmed one-frame noise is not retained across gaps.
  tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[&](const Track & t){
    return t.missed && t.hits<config_.min_hits;
  }),tracks_.end());
  for(auto index:high) if(!used[index] && detections[index].score>=config_.new_score && tracks_.size()<config_.max_tracks) {
    tracks_.push_back({detections[index],{},timestamp,next_id_++});
    if(config_.min_hits==1) output.push_back({index,tracks_.back().id});
  }
  return output;
}
}
