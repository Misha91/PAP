import sys
import os

inp = -1

def askInput():
    global inp
    validInput = False
    while(not validInput):
        try:
            inp = input("Press 0 for test verification, 1 for GCD program execution, 2 to exit.\nYour choice is ")
            inp = int(inp)
            if (inp >= 0 and inp <= 2):
                validInput = True
            else: raise Exception()

        except KeyboardInterrupt:
            print("\nEXIT")
            sys.exit()
        except:
            print("Incorrect input! Try again!")

def callTest():

    dummyCode = open("code/dummy.S", "r")
    dummyCode = dummyCode.readlines()
    for root, dirs, files in os.walk("code/test", topdown=False):
       for name in files:
           if name.endswith(".S"):
               print("************************************************")

               code = open("code/test/" + name, "r")
               code = code.readlines()


               outputFile = open("code/change_me.S", "w")

               for dl in dummyCode:
                   if 'nop' in dl:
                       for cl in code:
                           outputFile.write(cl)
                   outputFile.write(dl)

               outputFile.close()

               assert os.system("cd code;make >> make_res.txt") == 0, "CANNOT MAKE CODE!"
               assert os.system("cd code; cp change_me.dat ../memfile.dat") == 0, "CANNOT COPY DAT FILE!"

               assert os.system("make >> res.txt") == 0, "CANNOT MAKE MIPS CODE!"

               resExp = open("code/test/" + name.split(".")[0]+".res", "r")
               resExp = (resExp.readlines())[-1]
               resExp = resExp.split(",")

               res = open("res.txt", "r")
               res = (res.readlines())[-1]
               resLine = res.split(',')

               passNum = 0
               failNum = 0
               totalNum = 0

               for i in range(len(resExp)):
                   print(resExp[i].lstrip().rstrip() , resLine[i].lstrip().rstrip())
                   if resExp[i].lstrip().rstrip() == resLine[i].lstrip().rstrip():
                       passNum += 1
                   else:
                       failNum += 1
                   totalNum += 1

               print("Testing " + name)
               print("Passed = " + str(passNum) + "/" + str(totalNum) + " (" + str(round(float(float(passNum)/float(totalNum)), 2)*100 ) + "%)" )


    print("************************************************")
    print("FINISHED\n\n")
    print("Wanna do something else?")


def callGcd():
    print("************************************************")
    print("Preparing ...")
    dummyCode = open("code/dummy.S", "r")
    dummyCode = dummyCode.readlines()

    gcdCode = open("code/gcd.S", "r")
    gcdCode = gcdCode.readlines()
    var1 = (gcdCode[0].split(",")[-1]).rstrip().lstrip()
    var2 = (gcdCode[1].split(",")[-1]).rstrip().lstrip()
    outputFile = open("code/change_me.S", "w")

    for dl in dummyCode:
        if 'nop' in dl:
            for gl in gcdCode:
                outputFile.write(gl)
        outputFile.write(dl)

    outputFile.close()

    assert os.system("cd code;make >> make_res.txt") == 0, "CANNOT MAKE CODE!"
    assert os.system("cd code; cp change_me.dat ../memfile.dat") == 0, "CANNOT COPY DAT FILE!"

    print("Finished! Making mips code...")
    print("************************************************")
    print("See output at res.txt...")
    assert os.system("make >> res.txt") == 0, "CANNOT MAKE MIPS CODE!"
    res = open("res.txt", "r")
    res = res.readlines()

    resLine = res[-1]
    resLine = (resLine.split(",")[2]).lstrip()
    print("The result of GCD for " + var1 + " and " + var2 + " is " + str(resLine))

    print("************************************************")
    print("FINISHED\n\n")
    print("Wanna do something else?")

print("Welcome to MIPS in Verilog processor verification programm!")

while (inp != 2):
    askInput()
    if inp == 0:
        callTest()
    if inp == 1:
        callGcd()

print("EXIT")
