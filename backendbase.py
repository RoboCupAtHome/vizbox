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
        self.on_image = []

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

    def attach_image(self, callback):
        """
        Add a callback for when an Image is received
        :param callback: function accepting a base64-encoded image
        :return:
        """
        self.on_image += [callback]

    def detach_image(self, callback):
        """
        Remove a callback from when an Image is received
        :param callback:
        :return:
        """
        self.on_image.remove(callback)