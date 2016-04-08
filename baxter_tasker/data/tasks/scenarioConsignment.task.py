import tf_helper 
baxter.bb.bs.setScenario("consignment")

side='left'
baxter.bb.predefined_box='cover_door'
pose = tf_helper.PS('base',[0.1546346899564699, 1.0304861174516684, -0.019855684013959291],[0.0054851197231805711, 9.7554586005692235e-05, -0.99982683302243358, 0.017782235884603124])
baxter.frame.setTF('cover_door_'+side,pose)
baxter.frame.waitUntilFrameUpdate('cover_door_'+side)
baxter.scene.createPredefinedBox(baxter.bb.predefined_box+'_'+side,baxter.bb.predefined_box)


baxter.bb.predefined_box='wako'
pose = tf_helper.PS('base',[0.74770025622959835, -0.23438836622502551, -0.084207336289634518],[-0.00078447960482708656, 0.00075660505862314349, 0.7197784965069477, 0.69420294446705766])
baxter.frame.setTF('wako_'+side,pose)
baxter.frame.waitUntilFrameUpdate('wako_'+side)
baxter.scene.createPredefinedBox(baxter.bb.predefined_box+'_'+side,baxter.bb.predefined_box)
for drop_off in baxter.scene.boxes[baxter.bb.predefined_box][1].keys():
    pose = tf_helper.PS('wako_'+side,baxter.scene.boxes[baxter.bb.predefined_box][1][drop_off][0:3],baxter.scene.boxes[baxter.bb.predefined_box][1][drop_off][3:7])
    baxter.frame.setTF(drop_off+'_'+side,pose)

side='right'
baxter.bb.predefined_box='wako'
pose = tf_helper.PS('base',[0.74770025622959835, -0.23438836622502551, -0.084207336289634518],[-0.00078447960482708656, 0.00075660505862314349, 0.7197784965069477, 0.69420294446705766])
baxter.frame.setTF('wako_'+side,pose)
baxter.frame.waitUntilFrameUpdate('wako_'+side)
baxter.scene.createPredefinedBox(baxter.bb.predefined_box+'_'+side,baxter.bb.predefined_box)
for drop_off in baxter.scene.boxes[baxter.bb.predefined_box][1].keys():
    pose = tf_helper.PS('wako_'+side,baxter.scene.boxes[baxter.bb.predefined_box][1][drop_off][0:3],baxter.scene.boxes[baxter.bb.predefined_box][1][drop_off][3:7])
    baxter.frame.setTF(drop_off+'_'+side,pose)


baxter.bb.predefined_box='cover_small'
pose = tf_helper.PS('base',[-0.1154672952277938, -0.55082974198234735, -0.025436698134014071],[-0.0056229273529901473, -0.0058122728606557362, -0.69528037029319134, 0.71869312425898368])
baxter.frame.setTF('cover_small_'+side,pose)
baxter.frame.waitUntilFrameUpdate('cover_small_'+side)
baxter.scene.createPredefinedBox(baxter.bb.predefined_box+'_'+side,baxter.bb.predefined_box)


baxter.bb.predefined_box='switch'
pose = tf_helper.PS('base',[-0.61521669779638666, -0.40538291706201746, -0.090059295667480735],[-0.13683260106514308, -0.13745251039253467, -0.69211036675775872, 0.69524591829005244])
baxter.frame.setTF('switch_'+side,pose)
baxter.frame.waitUntilFrameUpdate('switch_'+side)
baxter.scene.createPredefinedBox(baxter.bb.predefined_box+'_'+side,baxter.bb.predefined_box)

baxter.scene.gripperLength["left"] = 0.11 
baxter.scene.attachGripper("left","suction",True)
baxter.scene.gripperLength["right"] = 0.125 
baxter.scene.attachGripper("right","electric",True)
baxter.scene.makeTables("scenarioConsignment")




