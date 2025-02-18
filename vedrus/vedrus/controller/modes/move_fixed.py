import time

from .safety_stop import ModeSafetyStop
from .parent import ModeParent

DEBUG = True

# Режим "Фиксированное движение"
class ModeMoveFixed(ModeParent):
    # время последнего safety::alarm
    lastAlarmTime = None

    # последовательно пойманные safety::alarm
    alarmCount = 0

    # Cтартовал ли режим? (движется ли сейчас?)
    # Начальные тики
    startLeft = None
    startRight = None

    # На сколько тиков ехать (+-)
    incLeft = 0
    incRight = 0

    # поймал safety.warning
    def warning(self, msg):
        pass

    # поймал safety.alarm
    def alarm(self, msg):
        self.lastAlarmTime = time.time()
        self.alarmCount += 1

        if DEBUG:
            self.node.get_logger().info('ModeMoveFixed: got alarm count='+ str(self.alarmCount))
        return

    def cycle(self):
        if self.startLeft == None:
            # вход в режим. Начало движения
            if DEBUG:
                self.node.get_logger().info('ModeMoveFixed: start to move')

            self.startLeft = self.node.motorLeft.position
            self.startRight = self.node.motorRight.position

            self.node.left(500 if self.incLeft > 0 else -500)
            self.node.right(500 if self.incRight > 0 else -500)
        else:
            # рабочий режим

            # дошёл до нужной позиции? Останови мотор
            if  (self.incLeft >  0 and self.node.motorLeft.position >= self.startLeft + self.incLeft) or \
                (self.incLeft <= 0 and self.node.motorLeft.position <= self.startLeft + self.incLeft):
                if DEBUG:
                    self.node.get_logger().info('ModeMoveFixed: stop left motor')
                self.node.left(0)

            if  (self.incRight >  0 and self.node.motorRight.position >= self.startRight + self.incRight) or \
                (self.incRight <= 0 and self.node.motorRight.position <= self.startRight + self.incRight):
                if DEBUG:
                    self.node.get_logger().info('ModeMoveFixed: stop right motor')
                self.node.right(0)

            # оба мотора остановлены? Выход из режима
            if self.node.leftSpeed == 0 and self.node.rightSpeed == 0:
                if DEBUG:
                    self.node.get_logger().info('ModeMoveFixed: end of mode')
                return route.out


        # случайная одиночная тревога за 0.5с. Игнорю
        if self.alarmCount == 1 and time.time() - self.lastAlarmTime >= 0.5:
            if DEBUG:
                self.node.get_logger().info('ModeMoveFixed: drop single alarm')

            self.alarmCount = 0

        # много тревоги. Остановка!
        if self.alarmCount > 1:
            if DEBUG:
                self.node.get_logger().info('ModeMoveFixed: got many alarms! Will stop')

            self.node.left(0)
            self.node.right(0)

            route = ModeSafetyStop(self.node)
            route.out = self

            return route

        #if DEBUG:
        #	node.get_logger().info('ModeMoveFixed: continue moving...')

        # ... или продолжить жить в этом режиме
        return self