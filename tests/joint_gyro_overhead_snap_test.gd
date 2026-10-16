extends SceneTree
# The arm overhead, where the anatomy's limits and the load limit change steeply: it must not
# turn over in jerks. Over the head and across (plane 130, elevation ~180, elbow 70), the pad's
# nose rocked: the load limit's steps used to swing the arm 30° round at 7.5° a frame. Overhead
# and swung fast out and back behind: the highest the arm can rise behind the back used to drop
# it 60° in eight frames. The elevation is asked to stay put; only the limits move it.
# Run: godot --headless --path . --script res://tests/joint_gyro_overhead_snap_test.gd

func _initialize():
	call_deferred("run")

func play(lab, pad_of):
	lab.CentrePad()
	var rng = RandomNumberGenerator.new()
	rng.seed = 7
	var worst = []
	for d in 7:
		worst.append(0.0)
	var prev = []
	for d in 7:
		prev.append(lab.Angle(0, d))
	for f in 420:
		lab.DrivePad(pad_of.call(f) + Vector3(rng.randf_range(-0.3, 0.3), rng.randf_range(-0.3, 0.3), rng.randf_range(-0.3, 0.3)), Vector2.ZERO, 0.0, false)
		lab._Process(1.0 / 60)
		for d in 7:
			if f > 90:
				worst[d] = max(worst[d], abs(lab.Angle(0, d) - prev[d]))
			prev[d] = lab.Angle(0, d)
	return worst

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	var across = play(lab, func(f): return Vector3(min(f / 60.0, 1.0) * 70.0 + (20.0 * sin((f - 60) * TAU / 120.0) if f > 60 else 0.0), -20.0 * min(f / 60.0, 1.0), 0))
	print("over the head and across, nose rocked: biggest step per joint ", across)
	var back = play(lab, func(f): return Vector3(min(f / 60.0, 1.0) * 77.0, (25.0 + 30.0 * sin((f - 60) * TAU / 50.0)) if f > 60 else 0.0, 25))
	print("overhead, swung fast out and back behind: biggest step per joint ", back)
	# The plane follows the pad's own fast swing (up to 7.5° a frame); the joints only the limits
	# move must not jump.
	var ok = across.max() < 3.0 and back[1] < 5.5 and back[2] < 3.0
	if ok:
		print("PASS overhead: the limits turn the arm smoothly")
		quit()
	else:
		push_error("Overhead the arm jumps where a limit moves it")
		quit(1)
