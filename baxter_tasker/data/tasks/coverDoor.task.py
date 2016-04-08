
move_params = [3,0.1,0.1,5]
box = "cover_door"
id = 0
dst_plan = "P"
side="left"
baxter.hlm.executePlan(box,side,id,dst_plan,move_params)

baxter.bb.bs.id["cover_door"] = -1
baxter.bb.bs.skip_first = False
while not rospy.is_shutdown() and baxter.hlm.stop() is False:
	baxter.bb.bs.doCoverDoor()
	if baxter.bb.bs.id["cover_door"] == 3:
	    break
	baxter.hlm.executePlan(box,side,id,dst_plan,move_params)
        







