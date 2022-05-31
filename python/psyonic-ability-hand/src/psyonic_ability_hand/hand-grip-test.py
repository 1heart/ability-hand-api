import time
from psyonic_ability_hand import log
from psyonic_ability_hand.hand import Hand, MockComm, Grip
from psyonic_ability_hand.io import SerialIO


def main():
    log.info("starting grasp test")

#    hand = Hand(MockComm())
    hand = Hand(SerialIO(), protocol_version=1)

    try:
#        hand.start()

        hand.set_grip(Grip.PowerGrasp)
        time.sleep(5)


    #     # hand.grasp(width=0, speed=.1)
    #     # time.sleep(10)

    #     # hand.grasp(width=1, speed=.1)
    #     # time.sleep(10)

    #     # hand.grasp(width=0, speed=.4)
    #     # time.sleep(6)

    #     # hand.grasp(width=.5, speed=.4)
    #     # time.sleep(6)

    #     # hand.grasp(width=0, speed=.3)
    #     # time.sleep(6)

    #     # hand.grasp(width=1, speed=.3)
    #     # time.sleep(6)

    #     # hand.grasp(width=0, speed=6)
    #     # time.sleep(10)


    #     # t = 0
    #     # while True:
    #     #     t += 1
    #     #     time.sleep(1)
    #     #     print(hand.stats())
    except KeyboardInterrupt:
        pass
    finally:
        pass
#        hand.stop()

    print("Hand stopped")
    print(hand.stats())


if __name__ == '__main__':
    main()