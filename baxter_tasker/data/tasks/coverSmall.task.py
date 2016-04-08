move_params = [1.5,0.1,0.1,5]

box = "cover_small"
id = 0
dst_plan = "P"
side="right"
baxter.hlm.executePlan(box,side,id,dst_plan,move_params)
dst_plan = "O"
baxter.hlm.executePlan(box,side,id,dst_plan,move_params)


baxter.bb.bs.id["cover_small"] = -1
baxter.bb.bs.skip_first = False
while not rospy.is_shutdown() and baxter.hlm.stop() is False:
	baxter.bb.bs.doCoverSmall()
	if baxter.bb.bs.id["cover_small"] == 3:
	    break
	







