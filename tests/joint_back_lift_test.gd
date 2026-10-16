extends SceneTree

func _initialize():
	call_deferred("run")

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.CentrePad()
	for i in 90:
		lab.DrivePad(Vector3(18, 35, 11), Vector2.ZERO, 0.0, false)
		lab._Process(1.0 / 60)
	var low_plane = lab.Target(0, 0)
	for i in 240:
		lab.DrivePad(Vector3(lerp(18.0, 60.67, min(i / 90.0, 1.0)), 35, 11), Vector2.ZERO, 0.0, false)
		lab._Process(1.0 / 60)
	print("Back lift: low plane %.1f; raised plane %.1f; elevation %.1f -> %.1f" % [low_plane, lab.Target(0, 0), lab.Target(0, 1), lab.Angle(0, 1)])
	if low_plane < -90 and lab.Target(0, 0) > -40 and lab.Angle(0, 1) > 100:
		print("PASS backswing can rise around the shoulder")
		quit()
	else:
		push_error("Back lift remains blocked")
		quit(1)
