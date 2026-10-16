extends SceneTree
# The sword's sound follows the motion: held still or moved calmly, no swish; a quick cut with
# the pad, a loud one. R2 pressed cracks once, and the elbow coming straight with R2 held snaps once; let go and
# pressed again, both again. Run: godot --headless --path . --script res://tests/joint_sword_sound_test.gd

func _initialize():
	call_deferred("run")

func step(lab, pad, r2):
	lab.DrivePad(pad, Vector2.ZERO, r2, false)
	lab._Process(1.0 / 60)

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.CentrePad()
	for i in 90:
		step(lab, Vector3.ZERO, 0.0)
	var still = lab.SwishLevel
	# An ordinary move: the pad's nose up 30° and back over two seconds — silent.
	var calm = 0.0
	for f in 120:
		step(lab, Vector3(30.0 * sin(f * PI / 120.0), 0, 0), 0.0)
		calm = max(calm, lab.SwishLevel)
	# A quick cut: the pad's nose up 40° and back down in a third of a second.
	var loudest = 0.0
	var fastest = 0.0
	for f in 20:
		step(lab, Vector3(40.0 * sin(f * PI / 20.0), -20.0 * sin(f * PI / 20.0), 0), 0.0)
		loudest = max(loudest, lab.SwishLevel)
		fastest = max(fastest, lab.TipSpeed)
	for i in 60:
		step(lab, Vector3.ZERO, 0.0)
	var cracks0 = lab.Cracks
	var snaps0 = lab.Snaps
	# R2 down in three frames and held: the crack, then the snap as the elbow straightens.
	for f in 40:
		step(lab, Vector3.ZERO, min(1.0, f / 3.0))
	var cracks1 = lab.Cracks - cracks0
	var snaps1 = lab.Snaps - snaps0
	for f in 40:
		step(lab, Vector3.ZERO, 0.0)
	for f in 40:
		step(lab, Vector3.ZERO, min(1.0, f / 3.0))
	var cracks2 = lab.Cracks - cracks0
	var snaps2 = lab.Snaps - snaps0
	print("calm move: swish %.2f; " % calm, "still swish %.2f; quick cut: tip %.1f m/s, swish %.2f; R2: cracks %d, snaps %d, again %d/%d" % [still, fastest, loudest, cracks1, snaps1, cracks2, snaps2])
	if still < 0.05 and calm < 0.05 and loudest > 0.5 and cracks1 == 1 and snaps1 == 1 and cracks2 == 2 and snaps2 == 2:
		print("PASS sword sound: swish with the tip's speed, a crack and a snap on each R2 cut")
		quit()
	else:
		push_error("The sword's sound does not follow the motion or R2")
		quit(1)
