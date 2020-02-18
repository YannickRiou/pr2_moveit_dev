# Init for spencer project


rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: false, adreamMocapHuman: true, toasterSimuHuman: true, pr2Robot: false, spencerRobot: false, toasterSimuRobot: false, toasterSimuObject: true, arObject: true, om2mObject: false, gazeboObject: false, mocapObject: false}"

#############
### AREA ###
#############

rosservice call /area_manager/add_area "myArea:
  id: 3
  name: 'room_l_living_room'
  myOwner: ''
  areaType: 'room'
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 0.0
  height: 0.0
  poly:
    points:
    - {x: 2.35, y: 9.1, z: 0.0}
    - {x: 9.4, y: 9.1, z: 0.0}
    - {x: 9.4, y: 5, z: 0.0}
    - {x: 2.35, y: 5, z: 0.0}
  zmin: 0.0
  zmax: 2.5
  enterHysteresis: 0.01
  leaveHysteresis: 0.01
  insideEntities_: ['']
  upcomingEntities_: ['']
  leavingEntities_: ['']"

###############
### OBJECTS ###
###############

##############
# Living Room
##############

# rosservice call /toaster_simu/add_entity "id: 'table_l_0'
# name: 'IKEA_table_TORSBY'
# type: 'object'
# ownerId: ''"
#
# rosservice call /toaster_simu/set_entity_pose "id: 'table_l_0'
# ownerId: ''
# type: 'object'
# pose:
#   position:
#     x: 3.8
#     y: 7.5
#     z: 0.0
#   orientation:
#     x: 0.0
#     y: 0.0
#     z: 0.0
#     w: 1.0"

rosservice call /toaster_simu/add_entity "id: 'milk_box_0'
name: 'MilkBox'
type: 'object'
ownerId: ''"

rosservice call /toaster_simu/set_entity_pose "id: 'milk_box_0'
ownerId: ''
type: 'object'
pose:
  position:
    x: 1.6
    y: -0.5  
    z: 0.80
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707"

# rosservice call /toaster_simu/add_entity "id: 'wheat_box_0'
# name: 'WheatBox'
# type: 'object'
# ownerId: ''"
#
# rosservice call /toaster_simu/set_entity_pose "id: 'wheat_box_0'
# ownerId: ''
# type: 'object'
# pose:
#   position:
#     x: 1.6
#     y: 0.20
#     z: 0.80
#   orientation:
#     x: 0.0
#     y: 0.0
#     z: 0.707
#     w: 0.707"
