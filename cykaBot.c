
#define setDriveL(pwr) motor[lyDrive] = \
    motor[mlDrive] = \
    pwr
#define setDriveR(pwr) motor[ryDrive] =
    motor[mrDrive] =
    pwr
#define setLift(pwr) motor[lLift] = \
  motor[rLift] = \
  pwr
#define setChain(pwr) motor[lChain] = \
  motor[rChain] = \
  pwr
#define setGoal(pwr) motor[lGoal] = \
  motor[rGoal] = \
  pwr

typedef enum Powers { L_DRIVE = 0, R_DRIVE = 1, LIFT = 2, CHAIN = 3, GOAL = 4 };
typedef enum Buttons { UP = 1, DOWN = 2, LEFT = 4, RIGHT = 8 };
typedef enum Joysticks { RX = 0, RY = 1, LY = 2, LX = 3 };

void setDrive(int lPwr, int rPwr) {
  motor[lyDrive] =
    motor[mlDrive] =
    lPwr;
  motor[ryDrive] =
    motor[mrDrive] =
    rPwr;
}

typedef struct {
  int targ,
    val,
    valLast,
    time,
    timeLast,
    dt,
    integLim,
    out;
  float kP,
    kI,
    kD,
    prop,
    integ,
    deriv,
    sum;
} Pid;

void initPid(Pid* pid, float kP, float kI, float kD, int integLim) {
  pid->val =
    pid->valLast =
    pid->timeLast =
    pid->integ =
    pid->dt =
    0;
  pid->integLim = fabs(integLim);
  pid->kP = kP;
  pid->kI = kI;
  pid->kD = kD;
}

void setTarg(Pid* pid, int targ) {
  pid->targ = targ;
  pid->integ = 0;
}

int upPid(Pid* pid, int val) {
  pid->valLast = pid->val;
  pid->val = val;
  pid->timeLast = pid->time;
  pid->time = nSysTime;
  pid->dt = pid->time - pid->timeLast;
  pid->errLast = pid->err;
  pid->err = pid->targ - pid->val;
  pid->prop = pid->err * pid->kP;
  pid->integ += pid->err * pid->kI * pid->dt;
  if(fabs(pid->integ) > pid->integLim)
    pid->integ = pid->integLim * sgn(pid->integ);
  pid->deriv = (pid->kD * (pid->err - pid->errLast)) / (pid->dt);
  pid->sum = pid->prop + pid->integ + pid->deriv;
  pid->out = round(pid->sum);
  if(fabs(pid->out) > 127)
    pid->out = 127 * sgn(pid->out);
  return pid->out;
}

Pid Pids[5];


const int GOAL_DOWN = -500,
  GOAL_UP = 500,
  LIFT_DOWN = -1000,
  LIFT_POS1 = -300,
  LIFT_COEFF = 500,
  CHAIN_GRAB = -180,
  CHAIN_DROP = 60;

const float kPs[5] = { .1, .1 , .1, .1, .1 },
  kIs[5] = { .001, .001 , .001, .001, .001 },
  kDs[5] = { .1, .1 , .1, .1, .1 };

task main() {
  static short mtrPwrs[5],
    sticks[4],
    stackHeight;
  static char btn7 = 0,
    btn7Last,
    btn8 = 0,
    btn8Last;
  static bool flip = false,
    liftPidOn = false,
    chainPidOn = false,
    goalPidOn = false;

  for(int i = 0; i < 5; i++)
    initPid(&Pids[i], kPs[i], kIs[i], kDs[i], 127);

  while(true) {
    for(int i = 0; i < 4; i++) {
      if(fabs(vexRT[i]) > 11)
        sticks[i] = vexRT[i];
      else
        sticks[i] = 0;
    }
    btn7Last = btn7;
    btn8Last = btn8;
    btn7 = 0;
    btn8 = 0;
    if(vexRT[Btn7U])
      btn7 += 1;
    if(vexRT[Btn7D])
      btn7 += 2;
    if(vexRT[Btn7L])
      btn7 += 4;
    if(vexRT[Btn7R])
      btn7 += 8;
    if(vexRT[Btn8U])
      btn8 += 1;
    if(vexRT[Btn8D])
      btn8 += 2;
    if(vexRT[Btn8L])
      btn8 += 4;
    if(vexRT[Btn8R])
      btn8 += 8;

    if((btn7 & UP) && !(btn7Last & UP))
      flip = !flip;
    if((btn7 & LEFT) && !(btn7Last & LEFT))
      liftPidOn = !liftPidOn;
    if((btn7 & DOWN) && !(btn7Last & DOWN))
      chainPidOn = !chainPidOn;
    if((btn8 & RIGHT) && !(btn8Last & RIGHT))
      goalPidOn = !goalPidOn;
    if((btn8 & LEFT) && !(btn8Last & LEFT))
      stackHeight++;
    if((btn7 & RIGHT) && !(btn7Last & RIGHT))
      stackHeight--;
    if(fabs(vexRT[AccelX]) > 10 || fabs(vexRT[AccelY]) > 10)
      stackHeight = 0;
    if(stackHeight < 0)
      stackHeight = 0;

    mtrPwrs[L_DRIVE] = sticks[LY] + sticks[RX];
    mtrPwrs[R_DRIVE] = sticks[LY] - sticks[RX];
    if(flip) {
      int swap = mtrPwrs[L_DRIVE];
      mtrPwrs[L_DRIVE] = -mtrPwrs[R_DRIVE];
      mtrPwrs[R_DRIVE] = -swap;
    }

    if(liftPidOn) { 
      if(vexRT[Btn5U] ^ vexRT[Btn5D]) {
        if(vexRT[Btn5U])
          setTarg(&Pids[LIFT], (LIFT_POS1 + (stackHeight * LIFT_COEFF)));
        else 
          setTarg(&Pids[LIFT], LIFT_DOWN);
      }
      mtrPwrs[LIFT] = upPid(&Pids[LIFT], SensorValue[liftPot]);
    }
    else if(vexRT[Btn5U] ^ vexRT[Btn5D]) {
      if(vexRT[Btn5U])
        mtrPwrs[LIFT] = 127;
      else
        mtrPwrs[LIFT] = -127;
    }
    else
      mtrPwrs[LIFT] = 0;

    if(chainPidOn) { 
      if(vexRT[Btn6U] ^ vexRT[Btn6D]) {
        if(vexRT[Btn6U])
          setTarg(&Pids[CHAIN], CHAIN_DROP);
        else 
          setTarg(&Pids[CHAIN], CHAIN_GRAB);
      }
      mtrPwrs[CHAIN] = upPid(&Pids[CHAIN], SensorValue[chainEnc]);
    }
    else if(vexRT[Btn6U] ^ vexRT[Btn6D]) {
      if(vexRT[Btn6U])
        mtrPwrs[CHAIN] = 127;
      else
        mtrPwrs[CHAIN] = -127;
    }
    else if(vexRT[Btn6U])
      release();
    else
      mtrPwrs[CHAIN] = 0;

    if(goalPidOn) { 
      if((btn8 & UP) ^ (btn8 & DOWN)) {
        if(btn8 & UP)
          setTarg(&Pids[GOAL], GOAL_UP);
        else 
          setTarg(&Pids[GOAL], GOAL_DOWN);
      }
      mtrPwrs[GOAL] = upPid(&Pids[GOAL], SensorValue[goalPot]);
    }
    else if((btn8 & UP) ^ (btn8 & DOWN)) {
      if(btn8 & UP)
        mtrPwrs[GOAL] = 127;
      else
        mtrPwrs[GOAL] = -127;
    }
    else
      mtrPwrs[GOAL] = 0;

    setDrive(mtrPwrs[L_DRIVE], mtrPwrs[R_DRIVE]);
    setLift(mtrPwrs[LIFT]);
    setChain(mtrPwrs[CHAIN]);
    setGoal(mtrPwrs[GOAL]);

    wait1Msec(20);
  }
}
