'''
Reference: https://pythonprogramminglanguage.com/text-to-speech/
           https://pyttsx.readthedocs.io/en/v1.0/engine.html#pyttsx.voice.Voice

'''
import pyttsx
import time
from PID import PID

test_voice = False

def play_sound():
    engine = pyttsx.init()
    engine.setProperty('rate', 150)
    engine.setProperty('voice', b'english-us')

    engine.runAndWait()

    rate = engine.getProperty('rate')
    volume = engine.getProperty('volume')
    voice = engine.getProperty('voice')         # get id
    voices = engine.getProperty('voices')
    engine.runAndWait()                         # GetProperty isn't updated until after you've used the runAndWait() method
                                                # issue on https://github.com/nateshmbhat/pyttsx3/issues/59

    print("Rate: {} Volume: {} Voice: {}".format(rate,volume,voice))

    if test_voice:
        # Test which voices is better
        for voice in voices:
            print("Current voice id is {}".format(voice.id))
            engine.setProperty('voice', voice.id)
            engine.say('Drawing.')

    engine.say('Finish Drawing.')
    time.sleep(0.5)
    engine.say('Oh yeah')
    time.sleep(0.5)
    engine.say('Oh yeah')
    time.sleep(0.5)
    engine.say('Oh yeah')
    engine.runAndWait()

def pid():
    # PID Parameters
    KP = 0
    KI = 0.1
    KD = 0
    SAMPLE_TIME = 0.01
    UPPER_LIMIT = 200
    LOWER_LIMIT = -100

    # pid = PID(1, 0.1, 0.05, setpoint=1)

    # Create PID 
    pid = [PID() for x in range(2)]

    for x in range(2):
        pid[x].sample_time = SAMPLE_TIME
        pid[x].tunings = (KP, KI, KD)
        pid[x].output_limit = (LOWER_LIMIT, UPPER_LIMIT)
        pid[x].setpoint = 512
        pid[x].auto_mode = False



if __name__ == "__main__":
    # play_sound()