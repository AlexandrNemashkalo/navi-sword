extends SceneTree
# Turning the pad winds the arm out and back or across. From the guard there is 50° of plane to
# go across but 175° out and back, so at one gain the pad turned 20° for the one and 70° for the
# other. Checks the plane asked for: the far end back is reached by 35° turned right, the far
# end across by 20° left, the same gain as before at the centre, and no step in between.
# Run: godot --headless --path . --script res://tests/joint_pad_swing_test.gd

func _initialize():
	call_deferred("run")

func plane_for(lab, right):
	lab.CentrePad()
	for i in 90:
		lab.DrivePad(Vector3(0, right, 0), Vector2.ZERO, 0.0, false)
	return lab.Target(0, 0)

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	var back = plane_for(lab, 35)
	var across = plane_for(lab, -20)
	var centre = (plane_for(lab, -1) - plane_for(lab, 1)) / 2
	var worst = 0.0
	var previous = plane_for(lab, -25)
	for r in range(-24, 41):
		var now = plane_for(lab, r)
		worst = max(worst, previous - now)
		if now > previous + 0.01:
			worst = 999
		previous = now
	print("35° right: plane %.1f, 20° left: %.1f, gain at the centre %.2f°/°, largest step per degree %.1f°" % [back, across, centre, worst])
	if back <= -94 and across >= 129 and abs(centre - 2.5) < 0.1 and worst < 8:
		print("PASS pad swing: both far ends within an easy turn")
		quit()
	else:
		push_error("The pad's swing is lopsided")
		quit(1)
