
b2Vec2 = Box2D.Common.Math.b2Vec2

ptm_ratio = 1500


contour = [
  [   new b2Vec2(277/ptm_ratio, 487/ptm_ratio)  ,  new b2Vec2(292/ptm_ratio, 463/ptm_ratio)  ,  new b2Vec2(335/ptm_ratio, 429/ptm_ratio)  ,  new b2Vec2(327/ptm_ratio, 556/ptm_ratio)  ,  new b2Vec2(303/ptm_ratio, 538/ptm_ratio)  ,  new b2Vec2(285/ptm_ratio, 521/ptm_ratio)  ,  new b2Vec2(276/ptm_ratio, 504/ptm_ratio)  ] ,
                                                [   new b2Vec2(292/ptm_ratio, 463/ptm_ratio)  ,  new b2Vec2(277/ptm_ratio, 487/ptm_ratio)  ,  new b2Vec2(282/ptm_ratio, 474/ptm_ratio)  ] ,
                                                [   new b2Vec2(335/ptm_ratio, 429/ptm_ratio)  ,  new b2Vec2(368/ptm_ratio, 405/ptm_ratio)  ,  new b2Vec2(411/ptm_ratio, 378/ptm_ratio)  ,  new b2Vec2(465/ptm_ratio, 350/ptm_ratio)  ,  new b2Vec2(534/ptm_ratio, 323/ptm_ratio)  ,  new b2Vec2(404/ptm_ratio, 609/ptm_ratio)  ,  new b2Vec2(360/ptm_ratio, 582/ptm_ratio)  ,  new b2Vec2(327/ptm_ratio, 556/ptm_ratio)  ] ,
                                                [   new b2Vec2(534/ptm_ratio, 323/ptm_ratio)  ,  new b2Vec2(596/ptm_ratio, 306/ptm_ratio)  ,  new b2Vec2(654/ptm_ratio, 295/ptm_ratio)  ,  new b2Vec2(610/ptm_ratio, 689/ptm_ratio)  ,  new b2Vec2(553/ptm_ratio, 674/ptm_ratio)  ,  new b2Vec2(492/ptm_ratio, 652/ptm_ratio)  ,  new b2Vec2(448/ptm_ratio, 632/ptm_ratio)  ,  new b2Vec2(404/ptm_ratio, 609/ptm_ratio)  ] ,
                                                [   new b2Vec2(709/ptm_ratio, 289/ptm_ratio)  ,  new b2Vec2(769/ptm_ratio, 288/ptm_ratio)  ,  new b2Vec2(833/ptm_ratio, 293/ptm_ratio)  ,  new b2Vec2(878/ptm_ratio, 299/ptm_ratio)  ,  new b2Vec2(716/ptm_ratio, 702/ptm_ratio)  ,  new b2Vec2(660/ptm_ratio, 697/ptm_ratio)  ,  new b2Vec2(610/ptm_ratio, 689/ptm_ratio)  ,  new b2Vec2(654/ptm_ratio, 295/ptm_ratio)  ] ,
                                                [   new b2Vec2(1147/ptm_ratio, 560/ptm_ratio)  ,  new b2Vec2(1122/ptm_ratio, 593/ptm_ratio)  ,  new b2Vec2(1098/ptm_ratio, 611/ptm_ratio)  ,  new b2Vec2(1060/ptm_ratio, 631/ptm_ratio)  ,  new b2Vec2(1018/ptm_ratio, 650/ptm_ratio)  ,  new b2Vec2(1056/ptm_ratio, 358/ptm_ratio)  ,  new b2Vec2(1162/ptm_ratio, 518/ptm_ratio)  ,  new b2Vec2(1157/ptm_ratio, 540/ptm_ratio)  ] ,
                                                [   new b2Vec2(1122/ptm_ratio, 593/ptm_ratio)  ,  new b2Vec2(1147/ptm_ratio, 560/ptm_ratio)  ,  new b2Vec2(1137/ptm_ratio, 577/ptm_ratio)  ] ,
                                                [   new b2Vec2(1146/ptm_ratio, 429/ptm_ratio)  ,  new b2Vec2(1156/ptm_ratio, 450/ptm_ratio)  ,  new b2Vec2(1162/ptm_ratio, 473/ptm_ratio)  ,  new b2Vec2(1162/ptm_ratio, 518/ptm_ratio)  ,  new b2Vec2(1056/ptm_ratio, 358/ptm_ratio)  ,  new b2Vec2(1086/ptm_ratio, 374/ptm_ratio)  ,  new b2Vec2(1111/ptm_ratio, 390/ptm_ratio)  ,  new b2Vec2(1130/ptm_ratio, 407/ptm_ratio)  ] ,
                                                [   new b2Vec2(921/ptm_ratio, 309/ptm_ratio)  ,  new b2Vec2(967/ptm_ratio, 322/ptm_ratio)  ,  new b2Vec2(1018/ptm_ratio, 341/ptm_ratio)  ,  new b2Vec2(1018/ptm_ratio, 650/ptm_ratio)  ,  new b2Vec2(938/ptm_ratio, 678/ptm_ratio)  ,  new b2Vec2(901/ptm_ratio, 687/ptm_ratio)  ,  new b2Vec2(716/ptm_ratio, 702/ptm_ratio)  ,  new b2Vec2(878/ptm_ratio, 299/ptm_ratio)  ] ,
                                                [   new b2Vec2(1162/ptm_ratio, 518/ptm_ratio)  ,  new b2Vec2(1162/ptm_ratio, 473/ptm_ratio)  ,  new b2Vec2(1164/ptm_ratio, 495/ptm_ratio)  ] ,
                                                [   new b2Vec2(1018/ptm_ratio, 650/ptm_ratio)  ,  new b2Vec2(1018/ptm_ratio, 341/ptm_ratio)  ,  new b2Vec2(1056/ptm_ratio, 358/ptm_ratio)  ] ,
                                                [   new b2Vec2(938/ptm_ratio, 678/ptm_ratio)  ,  new b2Vec2(1018/ptm_ratio, 650/ptm_ratio)  ,  new b2Vec2(980/ptm_ratio, 665/ptm_ratio)  ] ,
                                                [   new b2Vec2(829/ptm_ratio, 699/ptm_ratio)  ,  new b2Vec2(792/ptm_ratio, 702/ptm_ratio)  ,  new b2Vec2(753/ptm_ratio, 703/ptm_ratio)  ,  new b2Vec2(716/ptm_ratio, 702/ptm_ratio)  ,  new b2Vec2(901/ptm_ratio, 687/ptm_ratio)  ,  new b2Vec2(866/ptm_ratio, 694/ptm_ratio)  ]
                                                ]
contourCenter = new b2Vec2(820/ptm_ratio, 502/ptm_ratio)

arm1ContourConvex = [
                                                [   new b2Vec2(514/ptm_ratio, 554/ptm_ratio)  ,  new b2Vec2(628/ptm_ratio, 684/ptm_ratio)  ,  new b2Vec2(593/ptm_ratio, 681/ptm_ratio)  ,  new b2Vec2(559/ptm_ratio, 675/ptm_ratio)  ,  new b2Vec2(531/ptm_ratio, 667/ptm_ratio)  ,  new b2Vec2(499/ptm_ratio, 639/ptm_ratio)  ,  new b2Vec2(489/ptm_ratio, 598/ptm_ratio)  ,  new b2Vec2(496/ptm_ratio, 573/ptm_ratio)  ] ,
                                                [   new b2Vec2(489/ptm_ratio, 598/ptm_ratio)  ,  new b2Vec2(499/ptm_ratio, 639/ptm_ratio)  ,  new b2Vec2(490/ptm_ratio, 618/ptm_ratio)  ] ,
                                                [   new b2Vec2(499/ptm_ratio, 639/ptm_ratio)  ,  new b2Vec2(531/ptm_ratio, 667/ptm_ratio)  ,  new b2Vec2(510/ptm_ratio, 654/ptm_ratio)  ] ,
                                                [   new b2Vec2(514/ptm_ratio, 554/ptm_ratio)  ,  new b2Vec2(613/ptm_ratio, 530/ptm_ratio)  ,  new b2Vec2(776/ptm_ratio, 644/ptm_ratio)  ,  new b2Vec2(728/ptm_ratio, 666/ptm_ratio)  ,  new b2Vec2(679/ptm_ratio, 680/ptm_ratio)  ,  new b2Vec2(628/ptm_ratio, 684/ptm_ratio)  ] ,
                                                [   new b2Vec2(857/ptm_ratio, 419/ptm_ratio)  ,  new b2Vec2(875/ptm_ratio, 439/ptm_ratio)  ,  new b2Vec2(814/ptm_ratio, 619/ptm_ratio)  ,  new b2Vec2(776/ptm_ratio, 644/ptm_ratio)  ,  new b2Vec2(691/ptm_ratio, 473/ptm_ratio)  ,  new b2Vec2(777/ptm_ratio, 397/ptm_ratio)  ,  new b2Vec2(807/ptm_ratio, 397/ptm_ratio)  ,  new b2Vec2(834/ptm_ratio, 404/ptm_ratio)  ] ,
                                                [   new b2Vec2(889/ptm_ratio, 523/ptm_ratio)  ,  new b2Vec2(877/ptm_ratio, 550/ptm_ratio)  ,  new b2Vec2(851/ptm_ratio, 583/ptm_ratio)  ,  new b2Vec2(814/ptm_ratio, 619/ptm_ratio)  ,  new b2Vec2(875/ptm_ratio, 439/ptm_ratio)  ,  new b2Vec2(888/ptm_ratio, 465/ptm_ratio)  ,  new b2Vec2(893/ptm_ratio, 494/ptm_ratio)  ] ,
                                                [   new b2Vec2(752/ptm_ratio, 405/ptm_ratio)  ,  new b2Vec2(777/ptm_ratio, 397/ptm_ratio)  ,  new b2Vec2(691/ptm_ratio, 473/ptm_ratio)  ,  new b2Vec2(711/ptm_ratio, 440/ptm_ratio)  ,  new b2Vec2(732/ptm_ratio, 417/ptm_ratio)  ] ,
                                                [   new b2Vec2(776/ptm_ratio, 644/ptm_ratio)  ,  new b2Vec2(655/ptm_ratio, 508/ptm_ratio)  ,  new b2Vec2(691/ptm_ratio, 473/ptm_ratio)  ] ,
                                                [   new b2Vec2(776/ptm_ratio, 644/ptm_ratio)  ,  new b2Vec2(613/ptm_ratio, 530/ptm_ratio)  ,  new b2Vec2(655/ptm_ratio, 508/ptm_ratio)  ] ,
                                                [   new b2Vec2(543/ptm_ratio, 540/ptm_ratio)  ,  new b2Vec2(577/ptm_ratio, 538/ptm_ratio)  ,  new b2Vec2(514/ptm_ratio, 554/ptm_ratio)  ,  new b2Vec2(526/ptm_ratio, 546/ptm_ratio)  ]
											]

arm1JointAnchor = new b2Vec2(794/ptm_ratio, 496/ptm_ratio)
arm1Center = new b2Vec2(743/ptm_ratio, 535/ptm_ratio)

arm2ContourConvex = [
  [   new b2Vec2(308/ptm_ratio, 455/ptm_ratio)  ,  new b2Vec2(316/ptm_ratio, 548/ptm_ratio)  ,  new b2Vec2(298/ptm_ratio, 534/ptm_ratio)  ,  new b2Vec2(284/ptm_ratio, 521/ptm_ratio)  ,  new b2Vec2(277/ptm_ratio, 484/ptm_ratio)  ,  new b2Vec2(282/ptm_ratio, 472/ptm_ratio)  ,  new b2Vec2(294/ptm_ratio, 460/ptm_ratio)  ] ,
                                                [   new b2Vec2(277/ptm_ratio, 484/ptm_ratio)  ,  new b2Vec2(284/ptm_ratio, 521/ptm_ratio)  ,  new b2Vec2(275/ptm_ratio, 499/ptm_ratio)  ] ,
                                                [   new b2Vec2(613/ptm_ratio, 571/ptm_ratio)  ,  new b2Vec2(620/ptm_ratio, 589/ptm_ratio)  ,  new b2Vec2(622/ptm_ratio, 608/ptm_ratio)  ,  new b2Vec2(468/ptm_ratio, 642/ptm_ratio)  ,  new b2Vec2(402/ptm_ratio, 608/ptm_ratio)  ,  new b2Vec2(584/ptm_ratio, 546/ptm_ratio)  ,  new b2Vec2(601/ptm_ratio, 556/ptm_ratio)  ] ,
                                                [   new b2Vec2(607/ptm_ratio, 647/ptm_ratio)  ,  new b2Vec2(590/ptm_ratio, 662/ptm_ratio)  ,  new b2Vec2(548/ptm_ratio, 671/ptm_ratio)  ,  new b2Vec2(525/ptm_ratio, 664/ptm_ratio)  ,  new b2Vec2(499/ptm_ratio, 655/ptm_ratio)  ,  new b2Vec2(468/ptm_ratio, 642/ptm_ratio)  ,  new b2Vec2(622/ptm_ratio, 608/ptm_ratio)  ,  new b2Vec2(618/ptm_ratio, 629/ptm_ratio)  ] ,
                                                [   new b2Vec2(308/ptm_ratio, 455/ptm_ratio)  ,  new b2Vec2(356/ptm_ratio, 473/ptm_ratio)  ,  new b2Vec2(373/ptm_ratio, 485/ptm_ratio)  ,  new b2Vec2(402/ptm_ratio, 608/ptm_ratio)  ,  new b2Vec2(358/ptm_ratio, 580/ptm_ratio)  ,  new b2Vec2(337/ptm_ratio, 565/ptm_ratio)  ,  new b2Vec2(316/ptm_ratio, 548/ptm_ratio)  ] ,
                                                [   new b2Vec2(548/ptm_ratio, 671/ptm_ratio)  ,  new b2Vec2(590/ptm_ratio, 662/ptm_ratio)  ,  new b2Vec2(570/ptm_ratio, 670/ptm_ratio)  ] ,
                                                [   new b2Vec2(356/ptm_ratio, 473/ptm_ratio)  ,  new b2Vec2(308/ptm_ratio, 455/ptm_ratio)  ,  new b2Vec2(322/ptm_ratio, 454/ptm_ratio)  ,  new b2Vec2(337/ptm_ratio, 458/ptm_ratio)  ] ,
                                                [   new b2Vec2(584/ptm_ratio, 546/ptm_ratio)  ,  new b2Vec2(402/ptm_ratio, 608/ptm_ratio)  ,  new b2Vec2(517/ptm_ratio, 537/ptm_ratio)  ,  new b2Vec2(564/ptm_ratio, 539/ptm_ratio)  ] ,
                                                [   new b2Vec2(399/ptm_ratio, 500/ptm_ratio)  ,  new b2Vec2(402/ptm_ratio, 608/ptm_ratio)  ,  new b2Vec2(373/ptm_ratio, 485/ptm_ratio)  ] ,
                                                [   new b2Vec2(429/ptm_ratio, 514/ptm_ratio)  ,  new b2Vec2(402/ptm_ratio, 608/ptm_ratio)  ,  new b2Vec2(399/ptm_ratio, 500/ptm_ratio)  ] ,
                                                [   new b2Vec2(458/ptm_ratio, 525/ptm_ratio)  ,  new b2Vec2(402/ptm_ratio, 608/ptm_ratio)  ,  new b2Vec2(429/ptm_ratio, 514/ptm_ratio)  ] ,
                                                [   new b2Vec2(490/ptm_ratio, 532/ptm_ratio)  ,  new b2Vec2(402/ptm_ratio, 608/ptm_ratio)  ,  new b2Vec2(458/ptm_ratio, 525/ptm_ratio)  ] ,
                                                [   new b2Vec2(517/ptm_ratio, 537/ptm_ratio)  ,  new b2Vec2(402/ptm_ratio, 608/ptm_ratio)  ,  new b2Vec2(490/ptm_ratio, 532/ptm_ratio)  ]
]
arm2JointAnchor = new b2Vec2(554/ptm_ratio, 605/ptm_ratio)
arm2Center = new b2Vec2(447/ptm_ratio, 568/ptm_ratio)

head = [
  new b2Vec2(1030.545/ptm_ratio,496.364/ptm_ratio),
  117.004/ptm_ratio
]
