# ScopeUI
small gtk + app to plot data recieved over wifi

# Abstract 
  The project aims at creating wireless DSO module. Which will sample and the analog voltage levels with a TivaC development board 
  and send the data over a wifi link using CC3100 addon board . This particular repository contains the code for the sender . All the details of the project (both reciever and sender side ) is given in [Project Report](report/ee712_project_report.pdf)
  
  the code is written entirely in C. 
      ![Please read report/ee712_project_report](report/libraries_used.png)
     
 
      
The program is multithreaded to address any dead lock issue. Cairo library is used for drawing ui 
![Please read report/ee712_project_report](report/ui_snap.png)

# Dependencies 

        Its recommeded to use Code Composter Studio for editing and building the code . You can download it from [CCS download page](http://software-dl.ti.com/ccs/esd/documents/ccs_downloads.html). Please note there are additional steps to be followed in case of linux install ( These are given in the above web page)
	
	Tivaware 	[TivaC SDK download page](http://software-dl.ti.com/tiva-c/SW-TM4C/latest/index_FDS.html) 
	CC3100 SDK      [CC3100 SDK download page](http://www.ti.com/tool/download/CC3100SDK)

	You can install the packages in windows after that you can simply copy paste the files to any linux machine. For instance one can install them
	in wine and copy the files to ccs install path. After installation open the getting_started_with_wireless_ap project in CC3100 SDK /platform_examples 
	directory and replace the three files in this project root (main.c sl_common.h and tm4c123gh6pm_startup_ccs.c). Build the project and flash the code . 
	You need to set appropriate server ip and port in the project . Follow instructions in the project report . If you are using Linux to build the code, make sure to check all the SDK paths you might have to change the file path seperator from \ to / . Also check the SDK root .  


### code for reciever (linux machine) is available at [ScopeUI project](https://github.com/aswinpajayan/ScopeUI)
# Building and running the SCOPEUI project 

	use the make file provided in the project root
	
	make 		- for building the project

	make run 	- to launch the ui 

	click on the connect button . now the ui can start recieving UDP packets . You will have to get the ip address of the machine and update it on the sender 

### Project members 
	1. Ananda Kundu 
	2. Sunny Mehtha 
	3. Aswin P Ajayan 
