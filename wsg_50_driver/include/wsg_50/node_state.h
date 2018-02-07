#ifndef GRIPPER_GENERAL_STATE_H_
#define GRIPPER_GENERAL_STATE_H_

enum class NodeStateType : int32_t
{
  UNKOWN = -1,
  START = 0,
  CONNECTING = 1,
  NOMINAL = 2,
  ERROR = 3
};

class NodeState
{
public:
  void set(NodeStateType state)
  {
    this->state = state;
  }
  NodeStateType get()
  {
    return this->state;
  }

private:
  NodeStateType state;
};

#endif /* GRIPPER_GENERAL_STATE_H_ */
