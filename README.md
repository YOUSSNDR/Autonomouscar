# Autonomouscar

### Introdruction
Here you will find all our work on the project of an autonomous. The first goal of this car will be to be able to follow a moving target. (the target in this project will be a person detaining a key sending a signal to the car).

In the different directories of this repository you will find the different élement to build your own Autonomous car.

This car will have two motorised back wheels and two free wheels in the front.
It will be turning thanks to the motorised wheels.

Three captors will be used to triangulize the position of a fourth one. The car will be following it's position and a Lidar will be use for obstacle detction and avoidance.

### The car
#### Frame
All the info on the car frame will be found in the [CAO folder](https://github.com/YOUSSNDR/Autonomouscar/tree/main/CAO).

#### Electronic
For the different electronic connection, they will be available in the [Electronic](https://github.com/YOUSSNDR/Autonomouscar/tree/main/Electronic) repository.
The different electronic component needed for this project are the following:

|Module|pieces|Tension(V)|Intensity(A)|
|------|------|---------|-------|
|[DWM1001_DEV](https://www.mouser.fr/ProductDetail/Qorvo/DWM1001-DEV?qs=TiOZkKH1s2T4sar5INj0ew%3D%3D&srsltid=AfmBOorqy5ZnZvl6puMKr_-_Af3t4eY71g53g9BRfKSXkqYK-x9jgRT1)|3|5|0.5|
|[LIDAR](https://www.amazon.fr/Slamtec-RPLIDAR-num%C3%A9risation-bstacles-navigation/dp/B07TJW5SXF)|1|5|0.23|
|[MPU6050](https://www.gotronic.fr/art-module-6-dof-sen-mpu6050-31492.htm)|1|5|0.02|
|[MOTOR+encoder](https://www.gotronic.fr/art-kit-moteur-encodeur-fit0450-27583.htm)|2|6|0.35|
|[Raspberry](https://www.amazon.fr/Raspberry-Pi-4595-mod%C3%A8les-Go/dp/B09TTNF8BT/ref=asc_df_B09TTNF8BT/?tag=googshopfr-21&linkCode=df0&hvadid=701511851267&hvpos=&hvnetw=g&hvrand=5788597801172693301&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9054962&hvtargid=pla-1679572971964&psc=1&mcid=e9453b632e653cdca350de98083c3cb0&gad_source=1)|1|5|3|

The battery will have to supply a voltage of 6V and as we aim to fully use the robot for half an hour, We will need a capacity of at least 2,73A/h.(We need at least 16,38Wh). 
*(this is purely theoric)

The best batteries found are the following:

|Battery|Tension(V)|Capacity(mA/h)|Number needed|price(€)|
|-------|----------|--------------|-------------|-----|
|[RoaringTop](https://www.amazon.fr/dp/B0CQLPSYJH/ref=sspa_dk_detail_0?psc=1&pd_rd_i=B0CQLPSYJH&pd_rd_w=jautD&content-id=amzn1.sym.d15aafde-9691-4d5f-85f2-056701d026bf&pf_rd_p=d15aafde-9691-4d5f-85f2-056701d026bf&pf_rd_r=RZWWWTWBQBWE7Z58CRMW&pd_rd_wg=Bb1E2&pd_rd_r=d12ca1cb-8461-43d6-8207-998c9ebf3e62&s=electronics&sp_csd=d2lkZ2V0TmFtZT1zcF9kZXRhaWw)|7,4|3000/6000(2sold)|1|35|
|[AMZZN](https://www.amazon.fr/dp/B0CY4RR9TQ/ref=sspa_dk_detail_4?psc=1&pd_rd_i=B0CY4RR9TQ&pd_rd_w=iPaLO&content-id=amzn1.sym.2295e42d-cd2a-4ee8-906d-272916f85e0f&pf_rd_p=2295e42d-cd2a-4ee8-906d-272916f85e0f&pf_rd_r=AANTYGM66JGX3JZWP72S&pd_rd_wg=8gY4z&pd_rd_r=9e76b041-9a30-4f2f-a665-2bed7f3fdace&s=electronics&sp_csd=d2lkZ2V0TmFtZT1zcF9kZXRhaWwy)|7,4|2000/4000(2sold)|2|33|
|[EEMB](https://www.amazon.fr/EEMB-103395-Rechargeable-Navigation-Enregistreur/dp/B08215B4KK/ref=pd_bxgy_thbs_d_sccl_1/260-1578461-0157465?pd_rd_w=xq9lj&content-id=amzn1.sym.cff2227d-f2f4-453e-8d37-b217535012a4&pf_rd_p=cff2227d-f2f4-453e-8d37-b217535012a4&pf_rd_r=4MED1ABWRB22CEQ6X35Z&pd_rd_wg=cbtG7&pd_rd_r=156d6414-ba8d-4357-ae8b-3c748a8d8292&pd_rd_i=B08215B4KK&psc=1)|(2*)3,7|3700|2|30|

An other criteria for the choice will be the size of the batterie which will be seen in the [README]() in the CAO directory.

#### src

The [src](https://github.com/YOUSSNDR/Autonomouscar/tree/main/src) directory will be gathering all packages used in this project.

the autoflow package can be used to launch a simulation of the robot.
The robot package contain the programms to use the real robot.