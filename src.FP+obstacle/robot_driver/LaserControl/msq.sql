-- MySQL dump 10.13  Distrib 5.5.43, for debian-linux-gnu (i686)
--
-- Host: 192.168.1.172    Database: xrs2015
-- ------------------------------------------------------
-- Server version	5.5.41-0ubuntu0.12.04.1

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `rb_topologypoint`
--

DROP TABLE IF EXISTS `rb_topologypoint`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `rb_topologypoint` (
  `id` varchar(40) NOT NULL,
  `fname` varchar(50) DEFAULT NULL,
  `lasermapX` float DEFAULT NULL,
  `lasermapY` float DEFAULT NULL,
  `lasermapTh` float DEFAULT NULL,
  `rfid` varchar(50) DEFAULT NULL COMMENT 'RFID',
  `slowrfid` varchar(50) DEFAULT NULL COMMENT '加速卡RFID',
  `direction` varchar(50) DEFAULT NULL COMMENT '点的方向，取值为从上向下a，从左向右b。从下向上c，从右向左d',
  `x` int(11) DEFAULT NULL COMMENT '地图坐标x',
  `y` int(11) DEFAULT NULL COMMENT '地图坐标y',
  `fun` varchar(50) DEFAULT NULL,
  `substation` varchar(50) NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8 COMMENT='拓扑点';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `rb_topologypoint`
--

LOCK TABLES `rb_topologypoint` WRITE;
/*!40000 ALTER TABLE `rb_topologypoint` DISABLE KEYS */;
INSERT INTO `rb_topologypoint` VALUES ('1d','1d',-0.17,-0.02,-3.07,'05050027',NULL,'d',480,270,'turnpoint','test'),('2b','2b',7.93,-0.12,-0.07,'05050002',NULL,'b',960,270,'turnpoint','test'),('2c','2c',8.59,-0.77,1.55,'05050015',NULL,'c',960,270,'turnpoint','test'),('2d','2d',9.21,0.01,-3.13,'05050014',NULL,'d',960,270,'turnpoint','test'),('3b','3b',16.02,-0.07,-0.02,'05050021',NULL,'b',1440,270,'turnpoint','test'),('3c','3c',16.55,-0.68,1.63,'05050012',NULL,'c',1440,270,'turnpoint','test'),('4c','4c',-0.66,-5.11,1.5,'05050001',NULL,'c',480,540,'turnpoint','test'),('4d','4d',0.4,-5.15,-3.11,'05050025',NULL,'d',480,540,'turnpoint','test'),('5a','5a',8.67,-4.28,-1.62,'05050028',NULL,'a',960,540,'turnpoint','test'),('5b','5b',7.78,-5.02,0.03,'05050019',NULL,'b',960,540,'turnpoint','test'),('5c','5c',8.74,-5.58,1.56,'05050024',NULL,'c',960,540,'turnpoint','test'),('5d','5d',9.48,-4.93,-3.1,'05050008',NULL,'d',960,540,'turnpoint','test'),('6a','6a',15.92,-5.84,-1.5,'05050009',NULL,'a',1440,540,'turnpoint','test'),('6b','6b',16.11,-4.91,0.08,'05050003',NULL,'b',1440,540,'turnpoint','test'),('7a','7a',-0.31,-9.11,-1.59,'05050026',NULL,'a',480,810,'turnpoint','test'),('7d','7d',0.42,-9.84,-0.75,'05050004',NULL,'d',480,810,'turnpoint','test'),('8a','8a',8.6,-9.1,-1.58,'05050029',NULL,'a',960,810,'turnpoint','test'),('8b','8b',7.78,-9.73,0.01,'05050030',NULL,'b',960,810,'turnpoint','test'),('8d','8d',9.35,-9.62,-3.11,'05050018',NULL,'d',960,810,'turnpoint','test'),('9b','9b',16.02,-9.55,0.01,'05050020',NULL,'b',1440,810,'turnpoint','test'),('Ad','Ad',15.27,-9.52,-3.11,'05050013',NULL,'d',1340,810,NULL,'test'),('Bb','Bb',0.03,0,0.03,'05050017',NULL,'b',580,270,NULL,'test'),('home','home',0,0,0,'04050030',NULL,'b',960,810,'home','test'),('home1','home1',0,0,0,'04050031',NULL,'d',960,810,'home1','test'),('id1_a','p1',0,0,NULL,'11',NULL,NULL,NULL,NULL,NULL,'station-1'),('id1_b','p2',0,0,NULL,'12',NULL,NULL,NULL,NULL,NULL,'station-1'),('id2_a','p3',3,0,NULL,'21',NULL,NULL,NULL,NULL,NULL,'station-1'),('id2_b','p4',3,0,NULL,'22',NULL,NULL,NULL,NULL,NULL,'station-1'),('id2_c','p5',3,0,NULL,'23',NULL,NULL,NULL,NULL,NULL,'station-1'),('id3_a','p6',8,0,NULL,'31',NULL,NULL,NULL,NULL,NULL,'station-1'),('id3_b','p7',8,0,NULL,'32',NULL,'c',NULL,NULL,NULL,'station-1'),('id4_a','p8',0,4,NULL,'41',NULL,NULL,NULL,NULL,NULL,'station-1'),('id4_b','p9',0,4,NULL,'42',NULL,NULL,NULL,NULL,NULL,'station-1'),('id4_c','p10',0,4,NULL,'43',NULL,NULL,NULL,NULL,NULL,'station-1'),('id5_b','p12',3,4,NULL,'52',NULL,NULL,NULL,NULL,NULL,'station-1'),('id5_c','p13',3,4,NULL,'53',NULL,NULL,NULL,NULL,NULL,'station-1'),('id5_d','p14',3,4,NULL,'54',NULL,NULL,NULL,NULL,NULL,'station-1'),('id6_a','p15',8,4,NULL,'61',NULL,NULL,NULL,NULL,NULL,'station-1'),('id6_b','p16',8,4,NULL,'62',NULL,NULL,NULL,NULL,NULL,'station-1'),('id6_c','p17',8,4,NULL,'63',NULL,NULL,NULL,NULL,NULL,'station-1'),('id7_a','p18',0,8,NULL,'71',NULL,NULL,NULL,NULL,NULL,'station-1'),('id7_b','p19',0,8,NULL,'72',NULL,NULL,NULL,NULL,NULL,'station-1'),('id8_a','p20',3,8,NULL,'81',NULL,NULL,NULL,NULL,NULL,'station-1'),('id8_b','p21',3,8,NULL,'82',NULL,NULL,NULL,NULL,NULL,'station-1'),('id8_c','p22',3,8,NULL,'83',NULL,NULL,NULL,NULL,NULL,'station-1'),('id9_a','p23',8,8,NULL,'91',NULL,NULL,NULL,NULL,NULL,'station-1'),('id9_b','p24',8,8,NULL,'92',NULL,NULL,NULL,NULL,NULL,'station-1'),('idA','p25',2,0,NULL,'93',NULL,NULL,NULL,NULL,NULL,'station-1'),('idB','p26',6,0,NULL,'94',NULL,NULL,NULL,NULL,NULL,'station-1'),('idC','p27',2,4,NULL,'95',NULL,NULL,NULL,NULL,NULL,'station-1'),('idD','p28',5,4,NULL,'96',NULL,NULL,NULL,NULL,NULL,'station-1'),('idE','p29',8,2,NULL,'97',NULL,NULL,NULL,NULL,NULL,'station-1'),('idF','p30',8,5,NULL,'98',NULL,NULL,NULL,NULL,NULL,'station-1'),('id_p100','p100',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,'station-1'),('id_p101','p101',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,'station-1'),('id_p102','p102',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,'station-1'),('id_p103','p103',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,'station-1'),('id_p104','p104',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,'station-1'),('T1d','T1d',3.57,-9.74,2.24,'05050016',NULL,'d',600,810,NULL,'test'),('T2b','T2b',12.03,-4.94,-0.01,'05050010',NULL,'b',1200,540,NULL,'test'),('T3d','T3d',4.23,-0.02,3.13,'05050006',NULL,'d',800,270,NULL,'test'),('T4a','T4a',8.74,-2.2,-1.56,'05050011',NULL,'a',960,350,NULL,'test'),('T5b','T5b',5.36,-9.75,0.02,'05050023',NULL,'b',700,810,NULL,'test'),('T6c','T6c',8.52,-3.07,1.56,'05050007',NULL,'c',960,450,NULL,'test'),('T7d','T7d',13.74,-4.86,3.1,'05050022',NULL,'d',1100,540,NULL,'test'),('T8b','T8b',12.37,-9.58,0.06,'05050005',NULL,'b',1100,810,NULL,'test');
/*!40000 ALTER TABLE `rb_topologypoint` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2015-06-05 11:44:13
