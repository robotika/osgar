cd artf
python -m osgar.replay --module detector %1 
cd ..
python -m subt.check_phone %1
