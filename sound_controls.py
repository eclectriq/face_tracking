import os

sound_last_play = {}

def play_sound(s):
    if s not in sound_last_play or (s in sound_last_play and (datetime.datetime.now() - sound_last_play[s]).seconds > 1):
        os.system('nohup nvlc ' + s + ' --play-and-exit > /dev/null 2>&1 &')
        sound_last_play[s] = datetime.datetime.now()