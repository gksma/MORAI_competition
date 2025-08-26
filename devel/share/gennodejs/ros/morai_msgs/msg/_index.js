
"use strict";

let ManipulatorControl = require('./ManipulatorControl.js');
let VehicleSpec = require('./VehicleSpec.js');
let RadarDetection = require('./RadarDetection.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let DillyCmdResponse = require('./DillyCmdResponse.js');
let WoowaDillyStatus = require('./WoowaDillyStatus.js');
let RadarDetections = require('./RadarDetections.js');
let ExternalForce = require('./ExternalForce.js');
let Lamps = require('./Lamps.js');
let UGVServeSkidCtrlCmd = require('./UGVServeSkidCtrlCmd.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let PREvent = require('./PREvent.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let TOF = require('./TOF.js');
let VehicleCollision = require('./VehicleCollision.js');
let GPSMessage = require('./GPSMessage.js');
let FaultStatusInfo_Overall = require('./FaultStatusInfo_Overall.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let Conveyor = require('./Conveyor.js');
let IntersectionControl = require('./IntersectionControl.js');
let CtrlCmd = require('./CtrlCmd.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let GVStateCmd = require('./GVStateCmd.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let IntscnTL = require('./IntscnTL.js');
let CollisionData = require('./CollisionData.js');
let FaultInjection_Controller = require('./FaultInjection_Controller.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let RobotState = require('./RobotState.js');
let SkateboardStatus = require('./SkateboardStatus.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let PRStatus = require('./PRStatus.js');
let RobotOutput = require('./RobotOutput.js');
let WheelControl = require('./WheelControl.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let FaultStatusInfo = require('./FaultStatusInfo.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let ShipState = require('./ShipState.js');
let EventInfo = require('./EventInfo.js');
let EgoVehicleStatusExtended = require('./EgoVehicleStatusExtended.js');
let DillyCmd = require('./DillyCmd.js');
let FaultInjection_Response = require('./FaultInjection_Response.js');
let MapSpec = require('./MapSpec.js');
let ReplayInfo = require('./ReplayInfo.js');
let GVDirectCmd = require('./GVDirectCmd.js');
let FaultInjection_Sensor = require('./FaultInjection_Sensor.js');
let Obstacle = require('./Obstacle.js');
let ERP42Info = require('./ERP42Info.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let SyncModeAddObject = require('./SyncModeAddObject.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let Transforms = require('./Transforms.js');
let SkateboardCtrlCmd = require('./SkateboardCtrlCmd.js');
let SkidSteer6wUGVCtrlCmd = require('./SkidSteer6wUGVCtrlCmd.js');
let FaultStatusInfo_Sensor = require('./FaultStatusInfo_Sensor.js');
let SVADC = require('./SVADC.js');
let SaveSensorData = require('./SaveSensorData.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let SkidSteer6wUGVStatus = require('./SkidSteer6wUGVStatus.js');
let ObjectStatusListExtended = require('./ObjectStatusListExtended.js');
let ShipCtrlCmd = require('./ShipCtrlCmd.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let GeoVector3Message = require('./GeoVector3Message.js');
let CMDConveyor = require('./CMDConveyor.js');
let FaultStatusInfo_Vehicle = require('./FaultStatusInfo_Vehicle.js');
let VelocityCmd = require('./VelocityCmd.js');
let FaultInjection_Tire = require('./FaultInjection_Tire.js');
let MultiPlayEventResponse = require('./MultiPlayEventResponse.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let TrafficLight = require('./TrafficLight.js');
let ObjectStatus = require('./ObjectStatus.js');
let Obstacles = require('./Obstacles.js');
let GhostMessage = require('./GhostMessage.js');
let WaitForTick = require('./WaitForTick.js');
let MultiPlayEventRequest = require('./MultiPlayEventRequest.js');
let SensorPosControl = require('./SensorPosControl.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let ObjectStatusExtended = require('./ObjectStatusExtended.js');
let ObjectStatusList = require('./ObjectStatusList.js');

module.exports = {
  ManipulatorControl: ManipulatorControl,
  VehicleSpec: VehicleSpec,
  RadarDetection: RadarDetection,
  SyncModeInfo: SyncModeInfo,
  NpcGhostCmd: NpcGhostCmd,
  MoraiSrvResponse: MoraiSrvResponse,
  DillyCmdResponse: DillyCmdResponse,
  WoowaDillyStatus: WoowaDillyStatus,
  RadarDetections: RadarDetections,
  ExternalForce: ExternalForce,
  Lamps: Lamps,
  UGVServeSkidCtrlCmd: UGVServeSkidCtrlCmd,
  SyncModeResultResponse: SyncModeResultResponse,
  SyncModeCmdResponse: SyncModeCmdResponse,
  PREvent: PREvent,
  MoraiTLInfo: MoraiTLInfo,
  TOF: TOF,
  VehicleCollision: VehicleCollision,
  GPSMessage: GPSMessage,
  FaultStatusInfo_Overall: FaultStatusInfo_Overall,
  WaitForTickResponse: WaitForTickResponse,
  MultiEgoSetting: MultiEgoSetting,
  Conveyor: Conveyor,
  IntersectionControl: IntersectionControl,
  CtrlCmd: CtrlCmd,
  EgoVehicleStatus: EgoVehicleStatus,
  SyncModeRemoveObject: SyncModeRemoveObject,
  DdCtrlCmd: DdCtrlCmd,
  PRCtrlCmd: PRCtrlCmd,
  GVStateCmd: GVStateCmd,
  MapSpecIndex: MapSpecIndex,
  IntscnTL: IntscnTL,
  CollisionData: CollisionData,
  FaultInjection_Controller: FaultInjection_Controller,
  GetTrafficLightStatus: GetTrafficLightStatus,
  MoraiSimProcHandle: MoraiSimProcHandle,
  RobotState: RobotState,
  SkateboardStatus: SkateboardStatus,
  MoraiTLIndex: MoraiTLIndex,
  VehicleCollisionData: VehicleCollisionData,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  PRStatus: PRStatus,
  RobotOutput: RobotOutput,
  WheelControl: WheelControl,
  IntersectionStatus: IntersectionStatus,
  FaultStatusInfo: FaultStatusInfo,
  SetTrafficLight: SetTrafficLight,
  ShipState: ShipState,
  EventInfo: EventInfo,
  EgoVehicleStatusExtended: EgoVehicleStatusExtended,
  DillyCmd: DillyCmd,
  FaultInjection_Response: FaultInjection_Response,
  MapSpec: MapSpec,
  ReplayInfo: ReplayInfo,
  GVDirectCmd: GVDirectCmd,
  FaultInjection_Sensor: FaultInjection_Sensor,
  Obstacle: Obstacle,
  ERP42Info: ERP42Info,
  ScenarioLoad: ScenarioLoad,
  SyncModeCmd: SyncModeCmd,
  VehicleSpecIndex: VehicleSpecIndex,
  SyncModeAddObject: SyncModeAddObject,
  SyncModeSetGear: SyncModeSetGear,
  Transforms: Transforms,
  SkateboardCtrlCmd: SkateboardCtrlCmd,
  SkidSteer6wUGVCtrlCmd: SkidSteer6wUGVCtrlCmd,
  FaultStatusInfo_Sensor: FaultStatusInfo_Sensor,
  SVADC: SVADC,
  SaveSensorData: SaveSensorData,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  SkidSteer6wUGVStatus: SkidSteer6wUGVStatus,
  ObjectStatusListExtended: ObjectStatusListExtended,
  ShipCtrlCmd: ShipCtrlCmd,
  MoraiSimProcStatus: MoraiSimProcStatus,
  GeoVector3Message: GeoVector3Message,
  CMDConveyor: CMDConveyor,
  FaultStatusInfo_Vehicle: FaultStatusInfo_Vehicle,
  VelocityCmd: VelocityCmd,
  FaultInjection_Tire: FaultInjection_Tire,
  MultiPlayEventResponse: MultiPlayEventResponse,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  TrafficLight: TrafficLight,
  ObjectStatus: ObjectStatus,
  Obstacles: Obstacles,
  GhostMessage: GhostMessage,
  WaitForTick: WaitForTick,
  MultiPlayEventRequest: MultiPlayEventRequest,
  SensorPosControl: SensorPosControl,
  NpcGhostInfo: NpcGhostInfo,
  ObjectStatusExtended: ObjectStatusExtended,
  ObjectStatusList: ObjectStatusList,
};
