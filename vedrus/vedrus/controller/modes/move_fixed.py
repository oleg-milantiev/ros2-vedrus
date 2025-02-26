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

    speedLeft = 1000
    speedRight = 1000

    # поймал safety.warning
    def warning(self, msg):
        pass

    # поймал safety.alarm
    def alarm(self, msg):
        self.lastAlarmTime = time.time()
        self.alarmCount += 1

        if DEBUG:
            self.node.get_logger().info(f"ModeMoveFixed: got alarm count={self.alarmCount} by {msg.header.frame_id}, range={msg.range}m, azimuth={msg.azimuth}°")
        return

    def cycle(self):
        if self.startLeft == None:
            # вход в режим. Начало движения
            if DEBUG:
                self.node.get_logger().info(f"ModeMoveFixed: Start to move ({self.incLeft},{self.incRight})")

            self.startLeft = self.node.motorLeft.position
            self.startRight = self.node.motorRight.position

            if self.incLeft != 0:
                self.node.left(self.speedLeft if self.incLeft > 0 else -self.speedLeft)
            if self.incRight != 0:
                self.node.right(self.speedRight if self.incRight > 0 else -self.speedRight)
        else:
            # рабочий режим

            # дошёл до нужной позиции? Останови мотор
            if  self.node.leftSpeed != 0 and \
                ((self.incLeft > 0 and self.node.motorLeft.position >= self.startLeft + self.incLeft) or \
                (self.incLeft <= 0 and self.node.motorLeft.position <= self.startLeft + self.incLeft)):
                if DEBUG:
                    self.node.get_logger().info('ModeMoveFixed: stop left motor')
                self.node.left(0)

            if  self.node.rightSpeed != 0 and \
                ((self.incRight > 0 and self.node.motorRight.position >= self.startRight + self.incRight) or \
                (self.incRight <= 0 and self.node.motorRight.position <= self.startRight + self.incRight)):
                if DEBUG:
                    self.node.get_logger().info('ModeMoveFixed: stop right motor')
                self.node.right(0)

            # оба мотора остановлены? Выход из режима
            if self.node.leftSpeed == 0 and self.node.rightSpeed == 0:
                if DEBUG:
                    self.node.get_logger().info('ModeMoveFixed: end of mode')
                return self.out

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
            
            # сброс тревог для дальнейшего начала движения
            self.alarmCount = 0
            
            # уменьшить inc на пройденное расстояние
            self.incLeft  = max(0, self.incLeft  - (self.node.motorLeft.position  - self.startLeft))
            self.incRight = max(0, self.incRight - (self.node.motorRight.position - self.startRight))
            
            # сброс для дальнейшего начала движения
            self.startLeft = None
            self.startRight = None

            route = ModeSafetyStop(self.node)
            route.out = self

            if DEBUG:
                self.node.get_logger().info('ModeMoveFixed: route to ModeSafetyStop with return to self')

            return route

        #if DEBUG:
        #	node.get_logger().info('ModeMoveFixed: continue moving...')

        # ... или продолжить жить в этом режиме
        return self