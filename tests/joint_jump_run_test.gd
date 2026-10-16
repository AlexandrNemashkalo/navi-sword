extends SceneTree
# The run (L3) and the jump (✕). Running he reaches RunSpeed, the feet stepping in turn (never
# both at once) and neither more than a leg's reach from under him. Jumping from standing he
# sinks, rises about 0.35 m, comes down and lands, the whole jump under a second. Jumping from a
# run he flies on with it, the feet going with the body (no step in the air).
# Run: godot --headless --path . --script res://tests/joint_jump_run_test.gd

func _initialize():
	call_deferred("run")

func body(lab):
	for c in lab.get_children():
		if c is Node3D and c.find_children("*", "Skeleton3D", true, false).size() > 0:
			return c.global_position
	return Vector3.ZERO

func flat(v):
	return Vector3(v.x, 0, v.z)

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	for i in 60:
		lab._Process(1.0 / 60)
	# The run.
	lab.SprintInput = true
	lab.WalkInput = Vector2(0, -1)
	var both = 0
	var reach = 0.0
	var fastest = 0.0
	var previous = [lab.FootPosition(0), lab.FootPosition(1)]
	for f in 150:
		lab._Process(1.0 / 60)
		var now = [lab.FootPosition(0), lab.FootPosition(1)]
		if now[0].distance_to(previous[0]) > 0.002 and now[1].distance_to(previous[1]) > 0.002:
			both += 1
		previous = now
		fastest = max(fastest, flat(lab.WalkVelocity).length())
		if f > 60:
			for i in 2:
				reach = max(reach, flat(now[i] - body(lab)).length())
	# A jump from the run.
	var takeoff = flat(body(lab))
	lab.Jump()
	var offsets = []
	var flew = 0.0
	for f in 60:
		lab._Process(1.0 / 60)
		if lab.Airborne:
			offsets.append(flat(lab.FootPosition(0) - body(lab)))
	var landed_run = not lab.Airborne
	var air_run = flat(body(lab)) - takeoff
	var drift = 0.0
	for o in offsets:
		drift = max(drift, o.distance_to(offsets[0]))
	# Standing still, then a jump in place.
	lab.SprintInput = false
	lab.WalkInput = Vector2.ZERO
	for i in 90:
		lab._Process(1.0 / 60)
	lab.Jump()
	var highest = 0.0
	var air_frames = 0
	var lowest = 0.0
	var frames = 0
	for f in 90:
		lab._Process(1.0 / 60)
		highest = max(highest, lab.AirHeight)
		if lab.Airborne:
			air_frames += 1
		lowest = min(lowest, body(lab).y)
	print("run: top %.2f m/s, both feet moving %d frames, a foot at most %.2f m from under him; jump from the run: %.2f m on, feet drift %.3f m in the air, landed %s; jump in place: %.2f m high, %d frames in the air" % [fastest, both, reach, air_run.length(), drift, landed_run, highest, air_frames])
	if fastest > 2.3 and both == 0 and reach < 0.6 and air_run.length() > 1.0 and drift < 0.05 and landed_run and abs(highest - 0.35) < 0.05 and air_frames > 25 and air_frames < 50:
		print("PASS jump and run: a fast run in turn, a jump up and on with it, landing")
		quit()
	else:
		push_error("The run or the jump is wrong")
		quit(1)
