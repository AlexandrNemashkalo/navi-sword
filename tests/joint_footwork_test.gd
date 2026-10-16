extends SceneTree
func _initialize():
 call_deferred('run')
func run():
 var lab=load('res://scenes/joint_lab.tscn').instantiate()
 root.add_child(lab)
 lab.set_process(false)
 for i in range(100):lab._Process(1.0/60)
 var before=[lab.FootPosition(0),lab.FootPosition(1)]
 lab.Set(0,1,80.0)
 lab.Set(0,0,-70.0)
 var lifted=false
 for i in range(150):
  lab._Process(1.0/60)
  lifted=lifted or lab.get('FeetMoving')
 var distance=max(before[0].distance_to(lab.FootPosition(0)),before[1].distance_to(lab.FootPosition(1)))
 assert(lifted and distance>0.04,'Joint lab feet did not step')
 for i in range(100):lab._Process(1.0/60)
 assert(not lab.get('FeetMoving'),'Feet did not settle')
 var planted=[lab.FootPosition(0),lab.FootPosition(1)]
 for i in range(60):lab._Process(1.0/60)
 assert(planted[0].is_equal_approx(lab.FootPosition(0)) and planted[1].is_equal_approx(lab.FootPosition(1)),'Idle feet slide')
 print('PASS joint lab footwork: displacement ',distance)
 quit()
