from steppyr import StepperController, DIRECTION_CW, DIRECTION_CCW
from steppyr.profiles.accel import AccelProfile
from steppyr.drivers.stepdir import StepDirDriver

import asyncio
from contextlib import suppress
from concurrent.futures import ThreadPoolExecutor

stepper1 = StepperController(
    profile=AccelProfile(), 
    driver=StepDirDriver(
        dir_pin=6, 
        step_pin=5,
    )
)

stepper2 = StepperController(
    profile=AccelProfile(), 
    driver=StepDirDriver(
        dir_pin=24, 
        step_pin=23,
    )
)

stepper1.activate()
stepper2.activate()
stepper1.set_target_acceleration(200)
stepper1.set_target_speed(800)
stepper2.set_target_acceleration(200)
stepper2.set_target_speed(800)

print("Initialized. Use the following keys for control")
print("WASD: controlling X and Y motors")
print("   X: to quit the program")

loop = asyncio.get_event_loop() #建立一個Event Loop

def user_input():
    while True:
        try:
            c = input()[0]
        except:
            continue
        
        if c == 'w':
            stepper1.move(-800)
        elif c == 's':
            stepper1.move(800)
        elif c == 'a':
            stepper2.move(-800)
        elif c == 'd':
            stepper2.move(800)
        elif c == 'x':
            break  # Exit the while loop

async def main(): #定義main()為一個協同程序/協程(coroutine)
    evloop = asyncio.get_event_loop() #建立一個Event Loop
    user_task = evloop.run_in_executor(None, user_input) #
    task1 = asyncio.create_task(stepper1.run_forever()) 
    task2 = asyncio.create_task(stepper2.run_forever())
    
    await user_task #

    task1.cancel()
    task2.cancel()
    with suppress(asyncio.CancelledError): #強制結束task1, task2 忽略exception(Cancel error)
        await task1
        await task2
    stepper1.shutdown()
    stepper2.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
