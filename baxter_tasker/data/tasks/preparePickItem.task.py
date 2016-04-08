move_params = [1.5,0.1,0.1,5]

box = "cover_small"
id = 0
dst_plan = "P"
side="right"
baxter.hlm.executePlan(box,side,id,dst_plan,move_params)
dst_plan = "O"
baxter.hlm.executePlan(box,side,id,dst_plan,move_params)

baxter.bb.bs.renderer.fill()
baxter.bb.bs.showBox()
baxter.bb.bs.skip_first = True
baxter.bb.bs.id["switch"] = -1
while not rospy.is_shutdown() and baxter.hlm.stop() is False:
	baxter.bb.bs.preparePickNextItem()
	if baxter.bb.bs.id["switch"] == 3:
		break
















