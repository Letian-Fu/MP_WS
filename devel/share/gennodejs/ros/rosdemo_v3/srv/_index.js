
"use strict";

let SpeedL = require('./SpeedL.js')
let DI = require('./DI.js')
let DO = require('./DO.js')
let SetHoldRegs = require('./SetHoldRegs.js')
let PositiveSolution = require('./PositiveSolution.js')
let TCPSpeedEnd = require('./TCPSpeedEnd.js')
let ToolDOExecute = require('./ToolDOExecute.js')
let CP = require('./CP.js')
let GetInRegs = require('./GetInRegs.js')
let ModbusClose = require('./ModbusClose.js')
let SyncAll = require('./SyncAll.js')
let GetHoldRegs = require('./GetHoldRegs.js')
let InverseSolution = require('./InverseSolution.js')
let PowerOn = require('./PowerOn.js')
let GetAngle = require('./GetAngle.js')
let DigitalOutputs = require('./DigitalOutputs.js')
let Arc = require('./Arc.js')
let StartDrag = require('./StartDrag.js')
let MovJIO = require('./MovJIO.js')
let GetTerminal485 = require('./GetTerminal485.js')
let StopScript = require('./StopScript.js')
let GetPose = require('./GetPose.js')
let Continues = require('./Continues.js')
let RelMovJUser = require('./RelMovJUser.js')
let AccL = require('./AccL.js')
let SetTerminal485 = require('./SetTerminal485.js')
let MovJ = require('./MovJ.js')
let TCPSpeed = require('./TCPSpeed.js')
let JointMovJ = require('./JointMovJ.js')
let SpeedJ = require('./SpeedJ.js')
let Sync = require('./Sync.js')
let LoadSwitch = require('./LoadSwitch.js')
let RelMovLUser = require('./RelMovLUser.js')
let DOExecute = require('./DOExecute.js')
let AOExecute = require('./AOExecute.js')
let RobotMode = require('./RobotMode.js')
let GetErrorID = require('./GetErrorID.js')
let AI = require('./AI.js')
let PauseScript = require('./PauseScript.js')
let SetObstacleAvoid = require('./SetObstacleAvoid.js')
let GetSixForceData = require('./GetSixForceData.js')
let PayLoad = require('./PayLoad.js')
let RelMovJ = require('./RelMovJ.js')
let DisableRobot = require('./DisableRobot.js')
let StopDrag = require('./StopDrag.js')
let AO = require('./AO.js')
let EmergencyStop = require('./EmergencyStop.js')
let EnableRobot = require('./EnableRobot.js')
let MoveJog = require('./MoveJog.js')
let SetCollideDrag = require('./SetCollideDrag.js')
let Circle = require('./Circle.js')
let DOGroup = require('./DOGroup.js')
let StartPath = require('./StartPath.js')
let DIGroup = require('./DIGroup.js')
let MovL = require('./MovL.js')
let MovJExt = require('./MovJExt.js')
let AccJ = require('./AccJ.js')
let StartTrace = require('./StartTrace.js')
let ClearError = require('./ClearError.js')
let SpeedFactor = require('./SpeedFactor.js')
let SetArmOrientation = require('./SetArmOrientation.js')
let SetTerminalKeys = require('./SetTerminalKeys.js')
let RelMovLTool = require('./RelMovLTool.js')
let Wait = require('./Wait.js')
let MovLIO = require('./MovLIO.js')
let ServoP = require('./ServoP.js')
let GetCoils = require('./GetCoils.js')
let RelJointMovJ = require('./RelJointMovJ.js')
let StopmoveJog = require('./StopmoveJog.js')
let RelMovJTool = require('./RelMovJTool.js')
let Jump = require('./Jump.js')
let Tool = require('./Tool.js')
let GetPathStartPose = require('./GetPathStartPose.js')
let LimZ = require('./LimZ.js')
let ToolDI = require('./ToolDI.js')
let ModbusCreate = require('./ModbusCreate.js')
let HandleTrajPoints = require('./HandleTrajPoints.js')
let RelMovL = require('./RelMovL.js')
let RunScript = require('./RunScript.js')
let GetInBits = require('./GetInBits.js')
let pause = require('./pause.js')
let SetPayload = require('./SetPayload.js')
let SetCoils = require('./SetCoils.js')
let BrakeControl = require('./BrakeControl.js')
let User = require('./User.js')
let ContinueScript = require('./ContinueScript.js')
let SetSafeSkin = require('./SetSafeSkin.js')
let Arch = require('./Arch.js')
let StartFCTrace = require('./StartFCTrace.js')
let ResetRobot = require('./ResetRobot.js')
let SetCollisionLevel = require('./SetCollisionLevel.js')
let ServoJ = require('./ServoJ.js')
let ToolAI = require('./ToolAI.js')
let GetTraceStartPose = require('./GetTraceStartPose.js')
let ToolDO = require('./ToolDO.js')

module.exports = {
  SpeedL: SpeedL,
  DI: DI,
  DO: DO,
  SetHoldRegs: SetHoldRegs,
  PositiveSolution: PositiveSolution,
  TCPSpeedEnd: TCPSpeedEnd,
  ToolDOExecute: ToolDOExecute,
  CP: CP,
  GetInRegs: GetInRegs,
  ModbusClose: ModbusClose,
  SyncAll: SyncAll,
  GetHoldRegs: GetHoldRegs,
  InverseSolution: InverseSolution,
  PowerOn: PowerOn,
  GetAngle: GetAngle,
  DigitalOutputs: DigitalOutputs,
  Arc: Arc,
  StartDrag: StartDrag,
  MovJIO: MovJIO,
  GetTerminal485: GetTerminal485,
  StopScript: StopScript,
  GetPose: GetPose,
  Continues: Continues,
  RelMovJUser: RelMovJUser,
  AccL: AccL,
  SetTerminal485: SetTerminal485,
  MovJ: MovJ,
  TCPSpeed: TCPSpeed,
  JointMovJ: JointMovJ,
  SpeedJ: SpeedJ,
  Sync: Sync,
  LoadSwitch: LoadSwitch,
  RelMovLUser: RelMovLUser,
  DOExecute: DOExecute,
  AOExecute: AOExecute,
  RobotMode: RobotMode,
  GetErrorID: GetErrorID,
  AI: AI,
  PauseScript: PauseScript,
  SetObstacleAvoid: SetObstacleAvoid,
  GetSixForceData: GetSixForceData,
  PayLoad: PayLoad,
  RelMovJ: RelMovJ,
  DisableRobot: DisableRobot,
  StopDrag: StopDrag,
  AO: AO,
  EmergencyStop: EmergencyStop,
  EnableRobot: EnableRobot,
  MoveJog: MoveJog,
  SetCollideDrag: SetCollideDrag,
  Circle: Circle,
  DOGroup: DOGroup,
  StartPath: StartPath,
  DIGroup: DIGroup,
  MovL: MovL,
  MovJExt: MovJExt,
  AccJ: AccJ,
  StartTrace: StartTrace,
  ClearError: ClearError,
  SpeedFactor: SpeedFactor,
  SetArmOrientation: SetArmOrientation,
  SetTerminalKeys: SetTerminalKeys,
  RelMovLTool: RelMovLTool,
  Wait: Wait,
  MovLIO: MovLIO,
  ServoP: ServoP,
  GetCoils: GetCoils,
  RelJointMovJ: RelJointMovJ,
  StopmoveJog: StopmoveJog,
  RelMovJTool: RelMovJTool,
  Jump: Jump,
  Tool: Tool,
  GetPathStartPose: GetPathStartPose,
  LimZ: LimZ,
  ToolDI: ToolDI,
  ModbusCreate: ModbusCreate,
  HandleTrajPoints: HandleTrajPoints,
  RelMovL: RelMovL,
  RunScript: RunScript,
  GetInBits: GetInBits,
  pause: pause,
  SetPayload: SetPayload,
  SetCoils: SetCoils,
  BrakeControl: BrakeControl,
  User: User,
  ContinueScript: ContinueScript,
  SetSafeSkin: SetSafeSkin,
  Arch: Arch,
  StartFCTrace: StartFCTrace,
  ResetRobot: ResetRobot,
  SetCollisionLevel: SetCollisionLevel,
  ServoJ: ServoJ,
  ToolAI: ToolAI,
  GetTraceStartPose: GetTraceStartPose,
  ToolDO: ToolDO,
};
