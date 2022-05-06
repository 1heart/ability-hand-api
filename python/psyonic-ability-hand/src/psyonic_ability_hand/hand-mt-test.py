import time

from psyonic_ability_hand.hand import Hand, MockComm

from loguru import logger as log

def main():
    log.info("starting mt test")

    hand = Hand(MockComm())

    try:
        hand.start()

        t = 0
        while True:
            t += 1
            time.sleep(1)
            print(hand.stats())
    except KeyboardInterrupt:
        pass
    finally:
        hand.stop()

    print("Hand stopped")
    print(hand.stats())


if __name__ == '__main__':
    main()