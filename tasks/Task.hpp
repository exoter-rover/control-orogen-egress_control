/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef EGRESS_CONTROL_TASK_TASK_HPP
#define EGRESS_CONTROL_TASK_TASK_HPP

#include "egress_control/TaskBase.hpp"
#include "exoter_kinematics/ExoterEgressTypes.hpp"
#include "exoter_kinematics/ExoterEgressControl.hpp"

using namespace exoter_kinematics;

namespace egress_control {

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare a new task context (i.e., a component)

The corresponding C++ class can be edited in tasks/Task.hpp and
tasks/Task.cpp, and will be put in the egress_control namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','egress_control::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        ExoterEgressControl* egress_control;
    private:
        std::vector<int> last_button_values;
        std::vector<double> last_axes_values;
        std::vector<std::string> joint_commands_names;
        std::vector<std::string> joint_readings_names;
        bool deadmans_switch;
        bool kill_switch;
        bool constant_speed_mode;
        double constant_speed;
        bool swap_driving_direction;
        bool constant_height_offset_mode;
        double constant_height_offset;

        const double MAX_SPEED; // Maximum speed in body x-direction in m/s.

        /** Maps the button presses and joystick movements to the corresponding function calls in the egress control and realizes the intended human interface behaviour.
         * \param joystick_commands Raw commands from joystick containing axes and button values.
         */
        void evaluateJoystickCommands(const controldev::RawCommand joystick_commands);

        /** Maps the input joint readings to the order that is expected by egress control.
         * \param joint_readings Joint readings from the joint dispatcher.
         */
        void evaluateJointReadings(const base::samples::Joints joint_readings);

        /** Assembles the output joint command vector from the calculated joint positions and velocities.
         * \param position_commands Position commands computed by egress control.
         * \param velocity_commands Velocity commands computed by egress control.
         */
        base::commands::Joints assembleJointCommands(const std::vector<double> position_commands, const std::vector<double> velocity_commands);
    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "egress_control::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

