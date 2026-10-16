extends SceneTree
func _initialize():call_deferred('run')
func angles(lab):
 var result=[]
 for a in range(2):
  for d in range(7):result.append(lab.Angle(a,d))
 return result
func run():
 var lab=load('res://scenes/joint_lab.tscn').instantiate()
 root.add_child(lab);lab.set_process(false)
 for i in range(160):lab._Process(1.0/60)
 var hidden=0
 var moved=0
 var worst_ms=0.0
 var pose=[45,169,-44,87,89,7,-10]
 for d in range(7):lab.Set(0,d,float(pose[d]))
 for frame in range(500):
  var previous=angles(lab)
  var searching=lab.get('PlanningPose') or frame==0 or frame==12
  var depth=lab.Overlap()
  if frame==12:lab.Set(0,0,-40.0)
  var started=Time.get_ticks_usec()
  lab._Process(1.0/60)
  worst_ms=max(worst_ms,(Time.get_ticks_usec()-started)/1000.0)
  var actual=angles(lab)
  var delta=0.0
  for d in range(14):delta=max(delta,abs(actual[d]-previous[d]))
  if searching:
   hidden+=1
   if delta>0.00001:
    push_error('Search leaked into rendered arm');quit(1);return
  elif delta>0.00001:
   moved+=1
   if delta>2.501:
    push_error('Displayed pose exceeded motion speed');quit(1);return
   if lab.Overlap()>max(0.00051,depth+0.00002):
    push_error('Displayed route deepened collision');quit(1);return
 if hidden==0 or moved==0 or lab.get('AcceptedPosePaths')<1:
  push_error('No hidden search or accepted movement exercised');quit(1);return
 print('PASS hidden search, replaced input, bounded playback and contacts; hidden frames ',hidden,' moving frames ',moved,' max process ms ',worst_ms)
 quit()
