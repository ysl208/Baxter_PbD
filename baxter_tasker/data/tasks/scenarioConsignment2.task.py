
import tf_helper
baxter.bb.bs.setScenario("consignment2")



side='right'
baxter.bb.predefined_box='tray'
pose = tf_helper.PS('base',[0.38293053920821635, -0.39778458087374469, -0.28246407028194215],[0.022888166680721266, 0.023623672329561012, -0.69546156287771688, 0.71781005042514168])
baxter.frame.setTF('tray_'+side,pose)
baxter.frame.waitUntilFrameUpdate('tray_'+side)
baxter.scene.createPredefinedBox(baxter.bb.predefined_box+'_'+side,baxter.bb.predefined_box)
for drop_off in baxter.scene.boxes[baxter.bb.predefined_box][1].keys():
    pose = tf_helper.PS('tray_'+side,baxter.scene.boxes[baxter.bb.predefined_box][1][drop_off][0:3],baxter.scene.boxes[baxter.bb.predefined_box][1][drop_off][3:7])
    baxter.frame.setTF(drop_off+'_'+side,pose)






side='right'
baxter.bb.predefined_box='table'
pose = tf_helper.PS('base',[-0.53491911559227867, -0.92050744437875398, -0.11309150925807064],[3.5593248199914698e-05, -0.0051030476511323116, 0.0069746397476057953, 0.99996265532177853])
baxter.frame.setTF('table_'+side,pose)
baxter.frame.waitUntilFrameUpdate('table_'+side)
baxter.scene.createPredefinedBox(baxter.bb.predefined_box+'_'+side,baxter.bb.predefined_box)
for drop_off in baxter.scene.boxes[baxter.bb.predefined_box][1].keys():
    pose = tf_helper.PS('table_'+side,baxter.scene.boxes[baxter.bb.predefined_box][1][drop_off][0:3],baxter.scene.boxes[baxter.bb.predefined_box][1][drop_off][3:7])
    baxter.frame.setTF(drop_off+'_'+side,pose)
    baxter.scene.createPredefinedBox(drop_off+'_'+side,drop_off)

side='left'
baxter.bb.predefined_box='table'
pose = tf_helper.PS('base',[-0.56478246422707834, 0.49805433372471014, -0.11714112664287696],[2.7470888703158708e-05, -0.0073203017614400625, 0.0037525719722760142, 0.99996616474312006])
baxter.frame.setTF('table_'+side,pose)
baxter.frame.waitUntilFrameUpdate('table_'+side)
baxter.scene.createPredefinedBox(baxter.bb.predefined_box+'_'+side,baxter.bb.predefined_box)
for drop_off in baxter.scene.boxes[baxter.bb.predefined_box][1].keys():
    pose = tf_helper.PS('table_'+side,baxter.scene.boxes[baxter.bb.predefined_box][1][drop_off][0:3],baxter.scene.boxes[baxter.bb.predefined_box][1][drop_off][3:7])
    baxter.frame.setTF(drop_off+'_'+side,pose)
    baxter.scene.createPredefinedBox(drop_off+'_'+side,drop_off)



side='left'
baxter.bb.predefined_box='tray'
pose = tf_helper.PS('base',[0.36032766655682191, 0.42502524993921131, -0.22555930962017456],[0.011295831656057589, 0.011144160804420376, -0.71178019662511627, 0.70222301554383282])
baxter.frame.setTF('tray_'+side,pose)
baxter.frame.waitUntilFrameUpdate('tray_'+side)
baxter.scene.createPredefinedBox(baxter.bb.predefined_box+'_'+side,baxter.bb.predefined_box)
for drop_off in baxter.scene.boxes[baxter.bb.predefined_box][1].keys():
    pose = tf_helper.PS('tray_'+side,baxter.scene.boxes[baxter.bb.predefined_box][1][drop_off][0:3],baxter.scene.boxes[baxter.bb.predefined_box][1][drop_off][3:7])
    baxter.frame.setTF(drop_off+'_'+side,pose)


baxter.scene.gripperLength["left"] = 0.162 
baxter.scene.attachGripper("left","electric",True)
baxter.scene.gripperLength["right"] = 0.162 
baxter.scene.attachGripper("right","electric",True)

