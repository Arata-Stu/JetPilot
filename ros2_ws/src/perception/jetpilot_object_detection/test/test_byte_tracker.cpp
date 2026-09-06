#include "jetpilot_object_detection/byte_tracker.hpp"
#include <iostream>
#include <stdexcept>
using namespace jetpilot_object_detection;
void check(bool value) {if(!value) throw std::runtime_error("tracker regression failed");}
Detection box(float x=0, float score=0.9F, int cls=0) {return {cls,score,x,0,x+20,20};}
int main() {
  ByteTracker t;
  check(t.update({box()},0,"camera").empty());
  auto a=t.update({box(1)},0.1,"camera");check(a.size()==1);auto id=a[0].id;
  a=t.update({box(2,0.2F)},0.2,"camera");check(a.size()==1&&a[0].id==id);
  check(t.update({},0.3,"camera").empty());
  a=t.update({box(4)},0.4,"camera");check(a.size()==1&&a[0].id==id);
  check(t.update({box(4)},0.4,"camera").empty());
  check(t.update({box(4)},1.1,"camera").empty());
  a=t.update({box(4)},1.2,"camera");check(a.size()==1&&a[0].id!=id);id=a[0].id;
  check(t.update({box(4)},0,"camera").empty());
  a=t.update({box(4)},0.1,"camera");check(a.size()==1&&a[0].id!=id);
  check(t.update({box(4)},0.2,"other").empty());
  ByteTracker low;check(low.update({box(0,0.2F)},0,"camera").empty());check(low.size()==0);
  TrackerConfig cfg;cfg.min_hits=1;ByteTracker multi(cfg);
  a=multi.update({box(0),box(50)},0,"camera");check(a.size()==2);auto left=a[0].id,right=a[1].id;
  a=multi.update({box(49),box(1)},0.1,"camera");check(a.size()==2);
  for(const auto & match:a) check(match.id==(match.detection_index==0?right:left));
  a=multi.update({box(1,0.9F,1)},0.2,"camera");check(a.size()==1&&a[0].id!=left&&a[0].id!=right);
  ByteTracker lost(cfg);a=lost.update({box()},0,"camera");id=a[0].id;
  lost.update({},0.1,"camera");check(lost.update({box(0,0.2F)},0.2,"camera").empty());
  a=lost.update({box()},0.3,"camera");check(a.size()==1&&a[0].id==id);
  bool threw=false;try {cfg.low_score=0.9F;ByteTracker invalid(cfg);} catch(const std::invalid_argument &) {threw=true;}
  check(threw);
  cfg=TrackerConfig{};cfg.min_hits=1;cfg.max_tracks=2;
  ByteTracker bounded(cfg);
  check(bounded.update({box(0),box(40),box(80)},0,"camera").size()==2);
  check(bounded.size()==2);
  check(bounded.update({},1,"camera").empty());check(bounded.size()==0);
  check(bounded.update({box(0)},1.1,"").empty());check(bounded.size()==0);
  std::cout << "tracker lifecycle, low-score recovery, ordering, class and time isolation passed\n";
}
