##prompt: "Go to the bedroom"
                    returns: {"action": "go_to_goal", "params": {"location": {"type": "string", "value": "bedroom"}}}

                    prompt: "Go to the bedroom,then go to the kitchen"
                    returns: {"action": "sequence", "params": [{"action": "go_to_goal", "params": {"location": {"type": "string", "value": "bedroom"}}}, {"action": "go_to_goal", "params": {"location": {"type": "string", "value": "kitchen"}}}]}
import pyautogui
import time


def new_tab():
    pyautogui.hotkey('ctrl', 'shift' ,'t')
    time.sleep(2.0)
    pyautogui.typewrite('1')
    time.sleep(2.0)
    pyautogui.hotkey('\n')
    time.sleep(1.5)

def window():
    pyautogui.hotkey('ctrl','alt' ,'t')
    time.sleep(2.0)
    pyautogui.typewrite('1')
    time.sleep(2.0)
    pyautogui.hotkey('\n')
    time.sleep(1.5)

def source():
    pyautogui.typewrite('source devel/setup.bash')
    pyautogui.hotkey('\n')
    time.sleep(3.0)

time.sleep(8.0)
pyautogui.click(x=100, y=100)
pyautogui.hotkey('tab')
time.sleep(0.5)
pyautogui.typewrite('17858')

pyautogui.hotkey('tab')
time.sleep(0.5)
pyautogui.typewrite('Aa87578757')

pyautogui.hotkey('tab')
time.sleep(0.5)
pyautogui.hotkey('enter')
time.sleep(4.0)


# start nodejs
window()
pyautogui.typewrite('cd ~/456')
pyautogui.hotkey('\n')
time.sleep(1.0)
source()
pyautogui.typewrite('cd src/test_nodejs/')
pyautogui.hotkey('\n')
time.sleep(1.0)
pyautogui.typewrite('node wsserver.js')
pyautogui.hotkey('\n')
time.sleep(1.0)

# navigate
window()
pyautogui.typewrite('cd ~/456')
pyautogui.hotkey('\n')
source()
pyautogui.typewrite('roslaunch tdk_launch navigate_to_saved_points.launch')
pyautogui.hotkey('\n')
time.sleep(6)


