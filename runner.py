import sys
import os


print("************************************************")
print("Preparing ...")

assert os.system("cd code;make") == 0, "CANNOT MAKE CODE!"
assert os.system("cd code; cp change_me.dat ../memfile.dat") == 0, "CANNOT COPY DAT FILE!"

print("Finished! Making mips code...")
print("************************************************")

assert os.system("make") == 0, "CANNOT MAKE MIPS CODE!"

print("************************************************")
print("FINISHED")
