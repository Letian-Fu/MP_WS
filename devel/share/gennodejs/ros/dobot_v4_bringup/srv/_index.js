
"use strict";

let DragSensivity = require('./DragSensivity.js')
let TcpDashboard = require('./TcpDashboard.js')
let GetCurrentCommandId = require('./GetCurrentCommandId.js')
let DI = require('./DI.js')
let DO = require('./DO.js')
let SetHoldRegs = require('./SetHoldRegs.js')
let CalcTool = require('./CalcTool.js')
let CP = require('./CP.js')
let GetInRegs = require('./GetInRegs.js')
let ModbusClose = require('./ModbusClose.js')
let GetDO = require('./GetDO.js')
let GetHoldRegs = require('./GetHoldRegs.js')
let InverseSolution = require('./InverseSolution.js')
let PowerOn = require('./PowerOn.js')
let GetAngle = require('./GetAngle.js')
let SetOutputFloat = require('./SetOutputFloat.js')
let Arc = require('./Arc.js')
let StartDrag = require('./StartDrag.js')
let MovJIO = require('./MovJIO.js')
let GetPose = require('./GetPose.js')
let RelMovJUser = require('./RelMovJUser.js')
let GetInputInt = require('./GetInputInt.js')
let SetBackDistance = require('./SetBackDistance.js')
let AccL = require('./AccL.js')
let Stop = require('./Stop.js')
let MovJ = require('./MovJ.js')
let CalcUser = require('./CalcUser.js')
let InverseKin = require('./InverseKin.js')
let VelJ = require('./VelJ.js')
let GetStartPose = require('./GetStartPose.js')
let GetOutputInt = require('./GetOutputInt.js')
let RelMovLUser = require('./RelMovLUser.js')
let RobotMode = require('./RobotMode.js')
let GetOutputFloat = require('./GetOutputFloat.js')
let GetInputFloat = require('./GetInputFloat.js')
let GetErrorID = require('./GetErrorID.js')
let PositiveKin = require('./PositiveKin.js')
let AI = require('./AI.js')
let DisableRobot = require('./DisableRobot.js')
let StopDrag = require('./StopDrag.js')
let ToolDOInstant = require('./ToolDOInstant.js')
let AO = require('./AO.js')
let EmergencyStop = require('./EmergencyStop.js')
let EnableSafeSkin = require('./EnableSafeSkin.js')
let EnableRobot = require('./EnableRobot.js')
let SetOutputInt = require('./SetOutputInt.js')
let ModbusRTUCreate = require('./ModbusRTUCreate.js')
let MoveJog = require('./MoveJog.js')
let SetToolMode = require('./SetToolMode.js')
let Circle = require('./Circle.js')
let DOGroup = require('./DOGroup.js')
let StartPath = require('./StartPath.js')
let SetTool = require('./SetTool.js')
let GetDOGroup = require('./GetDOGroup.js')
let DIGroup = require('./DIGroup.js')
let MovL = require('./MovL.js')
let SetToolPower = require('./SetToolPower.js')
let AccJ = require('./AccJ.js')
let AOInstant = require('./AOInstant.js')
let ClearError = require('./ClearError.js')
let Pause = require('./Pause.js')
let SpeedFactor = require('./SpeedFactor.js')
let SetTool485 = require('./SetTool485.js')
let RelMovLTool = require('./RelMovLTool.js')
let StopMoveJog = require('./StopMoveJog.js')
let MovLIO = require('./MovLIO.js')
let ServoP = require('./ServoP.js')
let GetCoils = require('./GetCoils.js')
let RelJointMovJ = require('./RelJointMovJ.js')
let SetSafeWallEnable = require('./SetSafeWallEnable.js')
let RelMovJTool = require('./RelMovJTool.js')
let GetInputBool = require('./GetInputBool.js')
let Tool = require('./Tool.js')
let GetOutputBool = require('./GetOutputBool.js')
let DOInstant = require('./DOInstant.js')
let ToolDI = require('./ToolDI.js')
let ModbusCreate = require('./ModbusCreate.js')
let Continue = require('./Continue.js')
let RunScript = require('./RunScript.js')
let GetInBits = require('./GetInBits.js')
let GetAO = require('./GetAO.js')
let SetPayload = require('./SetPayload.js')
let SetUser = require('./SetUser.js')
let SetCoils = require('./SetCoils.js')
let BrakeControl = require('./BrakeControl.js')
let User = require('./User.js')
let SetOutputBool = require('./SetOutputBool.js')
let SetSafeSkin = require('./SetSafeSkin.js')
let SetCollisionLevel = require('./SetCollisionLevel.js')
let ServoJ = require('./ServoJ.js')
let ToolAI = require('./ToolAI.js')
let VelL = require('./VelL.js')
let SetPostCollisionMode = require('./SetPostCollisionMode.js')
let ToolDO = require('./ToolDO.js')

module.exports = {
  DragSensivity: DragSensivity,
  TcpDashboard: TcpDashboard,
  GetCurrentCommandId: GetCurrentCommandId,
  DI: DI,
  DO: DO,
  SetHoldRegs: SetHoldRegs,
  CalcTool: CalcTool,
  CP: CP,
  GetInRegs: GetInRegs,
  ModbusClose: ModbusClose,
  GetDO: GetDO,
  GetHoldRegs: GetHoldRegs,
  InverseSolution: InverseSolution,
  PowerOn: PowerOn,
  GetAngle: GetAngle,
  SetOutputFloat: SetOutputFloat,
  Arc: Arc,
  StartDrag: StartDrag,
  MovJIO: MovJIO,
  GetPose: GetPose,
  RelMovJUser: RelMovJUser,
  GetInputInt: GetInputInt,
  SetBackDistance: SetBackDistance,
  AccL: AccL,
  Stop: Stop,
  MovJ: MovJ,
  CalcUser: CalcUser,
  InverseKin: InverseKin,
  VelJ: VelJ,
  GetStartPose: GetStartPose,
  GetOutputInt: GetOutputInt,
  RelMovLUser: RelMovLUser,
  RobotMode: RobotMode,
  GetOutputFloat: GetOutputFloat,
  GetInputFloat: GetInputFloat,
  GetErrorID: GetErrorID,
  PositiveKin: PositiveKin,
  AI: AI,
  DisableRobot: DisableRobot,
  StopDrag: StopDrag,
  ToolDOInstant: ToolDOInstant,
  AO: AO,
  EmergencyStop: EmergencyStop,
  EnableSafeSkin: EnableSafeSkin,
  EnableRobot: EnableRobot,
  SetOutputInt: SetOutputInt,
  ModbusRTUCreate: ModbusRTUCreate,
  MoveJog: MoveJog,
  SetToolMode: SetToolMode,
  Circle: Circle,
  DOGroup: DOGroup,
  StartPath: StartPath,
  SetTool: SetTool,
  GetDOGroup: GetDOGroup,
  DIGroup: DIGroup,
  MovL: MovL,
  SetToolPower: SetToolPower,
  AccJ: AccJ,
  AOInstant: AOInstant,
  ClearError: ClearError,
  Pause: Pause,
  SpeedFactor: SpeedFactor,
  SetTool485: SetTool485,
  RelMovLTool: RelMovLTool,
  StopMoveJog: StopMoveJog,
  MovLIO: MovLIO,
  ServoP: ServoP,
  GetCoils: GetCoils,
  RelJointMovJ: RelJointMovJ,
  SetSafeWallEnable: SetSafeWallEnable,
  RelMovJTool: RelMovJTool,
  GetInputBool: GetInputBool,
  Tool: Tool,
  GetOutputBool: GetOutputBool,
  DOInstant: DOInstant,
  ToolDI: ToolDI,
  ModbusCreate: ModbusCreate,
  Continue: Continue,
  RunScript: RunScript,
  GetInBits: GetInBits,
  GetAO: GetAO,
  SetPayload: SetPayload,
  SetUser: SetUser,
  SetCoils: SetCoils,
  BrakeControl: BrakeControl,
  User: User,
  SetOutputBool: SetOutputBool,
  SetSafeSkin: SetSafeSkin,
  SetCollisionLevel: SetCollisionLevel,
  ServoJ: ServoJ,
  ToolAI: ToolAI,
  VelL: VelL,
  SetPostCollisionMode: SetPostCollisionMode,
  ToolDO: ToolDO,
};
