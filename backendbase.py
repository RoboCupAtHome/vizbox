def call_callbacks_in(cb_list, converter):
    def callback(message):
        converted = converter(message)
        for cb in cb_list:
            cb(converted)

    return callback


class BackendBase(object):
    @staticmethod
    def get_instance():
        raise NotImplementedError()

    def __init__(self):
        self.on_operator_text = []
        self.on_robot_text = []
        self.on_challenge_step = []

    def attach_operator_text(self, callback):
        self.on_operator_text += [callback]

    def attach_robot_text(self, callback):
        self.on_robot_text += [callback]

    def attach_challenge_step(self, callback):
        self.on_challenge_step += [callback]

    def detach_operator_text(self, callback):
        self.on_operator_text.remove(callback)

    def detach_robot_text(self, callback):
        self.on_robot_text.remove(callback)

    def detach_challenge_step(self, callback):
        self.on_challenge_step.remove(callback)

    def accept_command(self, command_text):
        raise NotImplementedError()


