extends SceneTree
# The sword in the left hand: the right-hander's pose mirrored. The pad turned and rolled right
# left-handed moves the arm as the pad turned and rolled left does right-handed (nose up alike);
# the shown model is an ordinary one (not turned inside out) whose bones stand where their pairs'
# mirror images do, the sword in its left hand; and the panel names the sword arm the left.
# Run: godot --headless --path . --script res://tests/joint_left_hand_test.gd

func _initialize():
	call_deferred("run")

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	var reference = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	root.add_child(reference)
	lab.set_process(false)
	reference.set_process(false)
	lab.LeftHanded = true
	var worst = 0.0
	var axes = [Vector3.RIGHT, Vector3.UP, Vector3.BACK]
	for axis in 3:
		lab.CentrePad()
		reference.CentrePad()
		for i in 60:
			lab.DrivePadAttitude(Quaternion.IDENTITY, Vector2.ZERO, 0.0, false, false, 0.0, 1.0 / 60)
			reference.DrivePad(Vector3.ZERO, Vector2.ZERO, 0.0, false)
		for i in 90:
			var degrees = min(i + 1, 30) * 0.5
			lab.DrivePadAttitude(Quaternion(axes[axis], deg_to_rad(degrees)), Vector2.ZERO, 0.0, false, false, 0.0, 1.0 / 60)
			# Right-handed this would be pad[axis] = degrees for nose up, -degrees for the turn and roll; mirrored the turn and roll flip.
			var pad = Vector3.ZERO
			pad[axis] = degrees
			reference.DrivePad(pad, Vector2.ZERO, 0.0, false)
			for d in 7:
				worst = max(worst, abs(lab.Target(0, d) - reference.Target(0, d)))
	# The pose is mirrored, not the model: the shown model is an ordinary one, each of its bones
	# where its pair of the unseen right-hander is mirrored, the sword in the shown left hand.
	for i in 5:
		lab._Process(1.0 / 60)
	var bone_error = 0.0
	for pair in [["hand.l", "hand.r"], ["lowerarm.l", "lowerarm.r"], ["upperarm.l", "upperarm.r"], ["hand.r", "hand.l"], ["foot.l", "foot.r"], ["foot.r", "foot.l"], ["head", "head"], ["spine_02", "spine_02"]]:
		bone_error = max(bone_error, lab.ShownBone(pair[0]).distance_to(lab.MirrorOf(lab.LogicBone(pair[1]))))
	var grip = lab.ShownSword.global_transform.origin.distance_to(lab.ShownBone("hand.l"))
	var hidden_grip = lab.Sword.global_transform.origin.distance_to(lab.LogicBone("hand.r"))
	var flipped = lab.ShownModelProper and lab.ShownSword.visible and not lab.Sword.visible and lab.ShownCamera.current and bone_error < 0.005 and abs(grip - hidden_grip) < 0.005
	print("pose mirror: bones off their mirrored places by %.4f m, sword from the left hand %.3f m (right-hander's %.3f), model ordinary %s" % [bone_error, grip, hidden_grip, lab.ShownModelProper])
	var named = false
	for l in lab.find_children("*", "Label", true, false):
		if l.text == "ЛЕВАЯ РУКА (с мечом)":
			named = true
	print("left hand: mirrored pad error %.5f, pose mirrored %s, sword arm named the left %s" % [worst, flipped, named])
	if worst < 0.02 and flipped and named:
		print("PASS left hand: the pad mirrored, the pose mirrored on an ordinary model, the arms named as seen")
		quit()
	else:
		push_error("The left hand is not the right-hander mirrored")
		quit(1)
