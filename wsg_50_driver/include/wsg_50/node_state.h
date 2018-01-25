#ifndef GRIPPER_GENERAL_STATE_H_
#define GRIPPER_GENERAL_STATE_H_

enum class NodeStateType: uint32_t {
	Start = 0,
	StartingGripValues = 1,
	StartingOpeningValues = 2,
	Nominal = 100,
	Error = 10000
};

class NodeState {
	public:
		void set(NodeStateType state) {
			this->state = state;
		}
		NodeStateType get() {
			return this->state;
		}

	private:
		NodeStateType state;
};


#endif /* GRIPPER_GENERAL_STATE_H_ */
