enum states
{
  ON, L, R, X, numStates
}; states currentState;

enum inputs
{
  lost, offLeft, offRight, onLine
}; inputs currentInput;

typedef struct state
{
  int checkTime;
  int output;
  states nextState[numStates];
} stateInfo;

stateInfo fsm[numStates] =
{
  {2, 0b11, {ON, L, R, ON}},
  {2, 0b10, {L, L, R, ON}},
  {2, 0b01, {R, L, R, ON}},
  {85, 0b00, {X, L, R, ON}}
};
