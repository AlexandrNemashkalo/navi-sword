extends SceneTree
# Walking on the left stick: pushed up for two seconds, the fighter goes away from the camera,
# the feet stepping in turn (never both at once), each staying put while the other steps, and
# neither left behind; the body rides the steps; the arm is left as it was. Let go, he stops
# within half a second and the feet settle. Sideways, he goes sideways, not turned.
# Run: godot --headless --path . --script res://tests/joint_walk_test.gd

func _initialize():
	call_deferred("run")

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	var camera = lab.find_children("*", "Camera3D", true, false)[0]
	for i in 60:
		lab._Process(1.0 / 60)
	var away = -camera.global_transform.basis.z
	away.y = 0
	away = away.normalized()
	var arm = []
	for d in 7:
		arm.append(lab.Angle(0, d))
	var start = lab.Walked
	lab.WalkInput = Vector2(0, -1)
	var steps = 0
	var both = 0
	var slid = 0.0
	var behind = 0.0
	var rode = 0.0
	var previous = [lab.FootPosition(0), lab.FootPosition(1)]
	var was_moving = [false, false]
	var model = lab.get_child(0)
	for c in lab.get_children():
		if c is Node3D and c.find_children("*", "Skeleton3D", true, false).size() > 0:
			model = c
	var heights = []
	for f in 120:
		lab._Process(1.0 / 60)
		var now = [lab.FootPosition(0), lab.FootPosition(1)]
		var moving = [now[0].distance_to(previous[0]) > 0.002, now[1].distance_to(previous[1]) > 0.002]
		if moving[0] and moving[1]:
			both += 1
		for i in 2:
			if moving[i] and not was_moving[i]:
				steps += 1
		was_moving = moving
		previous = now
		if f > 30:
			# Neither foot more than a stride behind the body.
			var body = model.global_position
			for i in 2:
				behind = max(behind, (body - now[i]).dot(away))
			heights.append(model.global_position.y)
	heights.sort()
	rode = heights[heights.size() - 1] - heights[0]
	var walked = (lab.Walked - start).dot(away)
	var sideways = (lab.Walked - start) - away * walked
	var arm_moved = 0.0
	for d in 7:
		arm_moved = max(arm_moved, abs(lab.Angle(0, d) - arm[d]))
	lab.WalkInput = Vector2.ZERO
	var stopped_at = -1
	for f in 90:
		lab._Process(1.0 / 60)
		if stopped_at < 0 and lab.WalkVelocity.length() < 0.01:
			stopped_at = f
	for f in 60:
		lab._Process(1.0 / 60)
	var settled = not lab.get("FeetMoving")
	# Sideways: pushed right, he goes right of the camera.
	var before = lab.Walked
	lab.WalkInput = Vector2(1, 0)
	for f in 60:
		lab._Process(1.0 / 60)
	lab.WalkInput = Vector2.ZERO
	var right = camera.global_transform.basis.x
	right.y = 0
	var side = (lab.Walked - before).normalized().dot(right.normalized())
	print("walked %.2f m away (%.2f off the line) in 2 s, %d steps, both feet moving %d frames, a foot at most %.2f m behind, ride %.3f m, arm moved %.2f°; stopped in %d frames, settled %s; right %.2f" % [
		walked, sideways.length(), steps, both, behind, rode, arm_moved, stopped_at, settled, side])
	if walked > 1.5 and sideways.length() < 0.1 and steps >= 6 and both == 0 and behind < 0.45 and rode > 0.005 \
		and arm_moved < 1 and stopped_at >= 0 and stopped_at < 30 and settled and side > 0.95:
		print("PASS walk: away from the camera in turn-by-turn steps, the body riding them; stops and settles")
		quit()
	else:
		push_error("Walking is off")
		quit(1)
