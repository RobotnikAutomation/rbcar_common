string CONTROL_MODE_AUTO=auto
string CONTROL_MODE_MANUAL=manual
string CONTROL_MODE_UNKNOWN=unknown

string SUBSTATE_INIT=init
string SUBSTATE_MANUAL=manual
string SUBSTATE_GO_MANUAL=go_manual
string SUBSTATE_AUTO=auto
string SUBSTATE_GO_AUTO=go_auto
string SUBSTATE_ERROR=error


# current control_mode
string control_mode
# current position of the control mode switch (hardware)
string control_switch
# Flat to show if hardware stop signal is active
bool hardware_stop

# overall state
string state
# subcomponents' state
string steering_controller_state
string traction_controller_state

# internal substate to manage changes in control mode
string substate
