extends SceneTree
# Stamina: a loaded pose held tires the sword arm; tired, the load limit comes down, so the held
# pose eases of itself (here the elbow unbends: the cheapest way under the limit), and the arm
# moves slower; let down, it recovers in a few seconds.
# Run: godot --headless --path . --script res://tests/joint_stamina_test.gd

func _initialize():
	call_deferred("run")

func hold(lab, seconds):
	for i in int(seconds * 60):
		lab._Process(1.0 / 60)

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	# A loaded pose the arm reaches freely: out to the side above the shoulder, elbow bent.
	lab.Set(0, 0, 20)
	lab.Set(0, 1, 110)
	lab.Set(0, 3, 60)
	hold(lab, 1)
	var fresh_load = lab.ArmLoad(0)
	var fresh_elevation = lab.Angle(0, 1)
	hold(lab, 14)
	var mid = lab.Fatigue
	hold(lab, 30)
	var tired = lab.Fatigue
	var tired_load = lab.ArmLoad(0)
	var tired_elevation = lab.Angle(0, 1)
	var limit = lab.LoadLimit
	print("arm out, raised: load %.1f fresh, fatigue %.2f after 15 s, %.2f after 45 s; tired: limit %.1f, load %.1f, elevation %.0f° (was %.0f°)" % [
		fresh_load, mid, tired, limit, tired_load, tired_elevation, fresh_elevation])
	# Let down: it recovers.
	for d in [0, 1, 3]:
		lab.Set(0, d, 0)
	hold(lab, 6)
	var rested = lab.Fatigue
	print("let down 6 s: fatigue %.2f" % rested)
	# Switched off, it never tires.
	lab.UseStamina = false
	lab.Set(0, 0, 20)
	lab.Set(0, 1, 110)
	lab.Set(0, 3, 60)
	hold(lab, 5)
	var off = lab.Fatigue
	var ok = fresh_load > 7 and mid > 0.1 and mid < tired and tired > 0.5 \
		and tired_load <= limit + 0.6 and tired_load < fresh_load - 1.5 and rested < 0.05 and off == 0
	if ok:
		print("PASS stamina: the held pose tires the arm and comes lower; let down, it recovers")
		quit()
	else:
		push_error("Stamina does not tire, limit or recover the arm")
		quit(1)
