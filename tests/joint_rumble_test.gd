extends SceneTree
# The pad vibrates with the postural load: not at all up to 7 points (category I and the light end
# of II), the light motor through the rest of II (to 12), stronger and the heavy motor coming in through III (to 25), full by
# 35 in IV — rising smoothly with the number, no step at the categories' bounds (only the
# onset: it starts at a twentieth, or the light motor would not turn at all). And only once the
# pose is held: a cut passing through a loaded pose does not buzz.
# Run: godot --headless --path . --script res://tests/joint_rumble_test.gd

func _initialize():
	call_deferred("run")

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	var ok = true
	var previous = -1.0
	var steepest = 0.0
	for tenth in range(0, 401):
		var load = tenth / 10.0
		var r = lab.RumbleAt(load)
		if r.x < previous - 1e-6 or r.y > r.x + 1e-6:
			ok = false
		# Past the onset at 7 (it starts at a twentieth, felt at once): no steps.
		if previous >= 0 and load > 7.15:
			steepest = max(steepest, r.x - previous)
		previous = r.x
	var line = ""
	for load in [3.0, 5.0, 8.0, 12.0, 18.0, 25.0, 30.0, 40.0]:
		var r = lab.RumbleAt(load)
		line += "%.0f: %.2f/%.2f  " % [load, r.x, r.y]
	print("load: light/heavy motor  ", line, " steepest per 0.1 point %.4f" % steepest)
	ok = ok and lab.RumbleAt(7.0).x == 0 and lab.RumbleAt(12.0).y == 0 and lab.RumbleAt(18.0).y > 0 \
		and lab.RumbleAt(35.0).x > 0.999 and steepest < 0.01
	# Held: the arm raised overhead, forward, elbow bent (a loaded guard) — the vibration
	# comes on over a little more than half a second, and goes once the arm comes down.
	lab.Set(0, 0, 90)
	lab.Set(0, 1, 150)
	lab.Set(0, 3, 110)
	var at = []
	for i in 90:
		lab._Process(1.0 / 60)
		at.append(lab.RumbleHeld)
	var loaded = lab.ArmLoad(0)
	for d in [0, 1, 3]:
		lab.Set(0, d, 0)
	for i in 90:
		lab._Process(1.0 / 60)
	print("load held %.1f: vibration on %.2f after 0.1 s, %.2f after 0.5 s, %.2f after 1.5 s; %.2f once eased" % [loaded, at[5], at[30], at[89], lab.RumbleHeld])
	ok = ok and loaded > 7 and at[5] < 0.1 and at[89] > 0.99 and lab.RumbleHeld == 0
	if ok:
		print("PASS rumble rises smoothly with the load, none in category I")
		quit()
	else:
		push_error("The rumble does not follow the load")
		quit(1)
