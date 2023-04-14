rm radio_fifo
mkfifo radio_fifo
mplayer -volume 50 "https://0n-80s.radionetz.de/0n-80s.mp3"  < radio_fifo
