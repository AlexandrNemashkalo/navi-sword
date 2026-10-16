extends SceneTree
func _initialize():call_deferred('run')
func run():
 var lab=load('res://scenes/joint_lab.tscn').instantiate()
 root.add_child(lab);lab.set_process(false)
 var pose=[45,70,0,80,0,-20,-10]
 for d in range(7):lab.Set(0,d,float(pose[d]))
 for i in range(400):lab._Process(1.0/60)
 var fast=lab.get('DirectPoseFrames')
 var changed=0
 var durations=[]
 var max_error=0.0
 for frame in range(240):
  var goal=80.0+5.0*sin(frame*0.04)
  lab.Set(0,3,goal)
  var previous=lab.Angle(0,3)
  var start=Time.get_ticks_usec()
  lab._Process(1.0/60)
  durations.append((Time.get_ticks_usec()-start)/1000.0)
  if abs(lab.Angle(0,3)-previous)>0.001:changed+=1
  max_error=max(max_error,abs(lab.Angle(0,3)-round(goal)))
 durations.sort()
 print('continuous input: fast frames ',lab.get('DirectPoseFrames')-fast,' changed ',changed,' max elbow error ',max_error,' median ms ',durations[120],' p95 ms ',durations[228])
 if lab.get('DirectPoseFrames')-fast<20 or max_error>2.0:
  push_error('Live input starved or lagged');quit(1);return
 print('PASS continuous input response')
 quit()
