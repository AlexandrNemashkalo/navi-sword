extends SceneTree
func _initialize():call_deferred('run')
func run():
 var lab=load('res://scenes/joint_lab.tscn').instantiate()
 root.add_child(lab);lab.set_process(false)
 lab.set('SearchCpuBudgetMs',0.01)
 var before=[]
 for a in range(2):
  for d in range(7):before.append(lab.Angle(a,d))
 lab.Set(0,1,170.0)
 for frame in range(100):
  lab._Process(1.0/60)
  for a in range(2):
   for d in range(7):
    if abs(lab.Angle(a,d)-before[a*7+d])>0.00001:
     push_error('Timed out search moved visible pose');quit(1);return
 if not lab.get('SearchTimedOut') or lab.get('PlanningPose'):
  push_error('Search timeout not enforced');quit(1);return
 var cpu=lab.get('LastSearchCpuMs')
 for i in range(60):lab._Process(1.0/60)
 if cpu!=lab.get('LastSearchCpuMs'):
  push_error('Timed out search restarted without input');quit(1);return
 print('PASS timeout preserves pose and does not retry')
 quit()
