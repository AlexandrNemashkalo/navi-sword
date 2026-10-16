extends SceneTree
func _initialize():call_deferred('run')
func run():
 var lab=load('res://scenes/joint_lab.tscn').instantiate()
 root.add_child(lab);lab.set_process(false)
 var poses=[[45,169,-44,87,89,7,-10],[130,80,80,130,90,45,10],[-95,130,-80,120,-90,-45,-25],[90,30,80,130,90,45,10]]
 for pose in poses:
  for d in range(7):lab.Set(0,d,float(pose[d]))
  var started=Time.get_ticks_usec()
  for i in range(600):lab._Process(1.0/60)
  var before=[]
  for a in range(2):
   for d in range(7):before.append(lab.Angle(a,d))
  var depth=lab.Overlap()
  var drift=0.0
  for i in range(120):
   lab._Process(1.0/60)
   for a in range(2):
    for d in range(7):drift=max(drift,abs(lab.Angle(a,d)-before[a*7+d]))
  print('pose ',pose,' settled ',lab.get('PoseSettled'),' drift ',drift,' overlap ',depth,' ms ',(Time.get_ticks_usec()-started)/1000.0)
  if drift>0.05 or not lab.get('PoseSettled'):
   push_error('Blocked pose did not settle');quit(1);return
 var previous=lab.Angle(0,1)
 # Return to a reachable guard. Lowering only the shoulder while keeping
 # extreme wrist/elbow targets is allowed to be rejected by path validation.
 for d in range(7):lab.Set(0,d,float([0,0,0,0,0,-20,-10][d]))
 for i in range(180):lab._Process(1.0/60)
 if abs(lab.Angle(0,1)-previous)<1:
  push_error('Solver did not resume after new input');quit(1);return
 print('PASS blocked pose stability and resumed input')
 quit()

