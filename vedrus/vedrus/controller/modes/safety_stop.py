import time
from .parent import ModeParent

DEBUG = True

# Режим "Останов перед препятствием"
class ModeSafetyStop(ModeParent):

    started = None

    # поймал safety.warning
    def warning(self, msg):
        pass

    # поймал safety.alarm
    def alarm(self, msg):
        if DEBUG:
            self.node.get_logger().info(f"ModeSafetyStop: got alarm by {msg.header.frame_id}, range={msg.range}m, azimuth={msg.azimuth}°")

        self.lastAlarmTime = time.time()
        return

    def cycle(self):
        self._statusInit()
        self._statusAppend('mode', 'SafetyStop')

        if self.started == None:
            self.started = True
            self.node.left(0)
            self.node.right(0)

        # выход из стопа в self.out после 2 секунд без safety::alarm
        if time.time() - self.lastAlarmTime > 2:
            if DEBUG:
                self.node.get_logger().info(f"ModeSafetyStop: alarm 2 seconds ago. Route out to {self.out.__class__.__name__}")

            self._statusAppend('action', 'Route out')
            self._statusPublish()

            if self.out:
                return self.out
            else:
                self.node.get_logger().info(f"ModeSafetyStop: No out route! Will wait")
                return self

        #if DEBUG:
        #	self.node.get_logger().info('ModeSafetyStop: continue stay...')

        self._statusAppend('action', 'Waiting')
        self._statusPublish()

        # ... или продолжить жить в этом режиме
        return self
