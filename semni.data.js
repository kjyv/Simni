// Generated by CoffeeScript 1.3.3
var arm1Contour, arm1ContourConvex, arm2Contour, arm2ContourConvex, b2Vec2, contour, ptm_ratio;

b2Vec2 = Box2D.Common.Math.b2Vec2;

contour = new Array(new b2Vec2(0.004710, 0.080588), new b2Vec2(0.004020, 0.079928), new b2Vec2(0.003360, 0.079208), new b2Vec2(0.002730, 0.078458), new b2Vec2(0.002190, 0.077648), new b2Vec2(0.001680, 0.076808), new b2Vec2(0.001260, 0.075938), new b2Vec2(0.000870, 0.075038), new b2Vec2(0.000570, 0.074107), new b2Vec2(0.000330, 0.073177), new b2Vec2(0.000150, 0.072217), new b2Vec2(0.000030, 0.071257), new b2Vec2(0.000000, 0.070267), new b2Vec2(0.000000, 0.069307), new b2Vec2(0.000090, 0.068347), new b2Vec2(0.000240, 0.067357), new b2Vec2(0.000480, 0.066427), new b2Vec2(0.000780, 0.065497), new b2Vec2(0.001110, 0.064596), new b2Vec2(0.001530, 0.063696), new b2Vec2(0.002010, 0.062856), new b2Vec2(0.002550, 0.062046), new b2Vec2(0.003120, 0.061266), new b2Vec2(0.003780, 0.060516), new b2Vec2(0.004470, 0.059856), new b2Vec2(0.013801, 0.051875), new b2Vec2(0.026613, 0.042244), new b2Vec2(0.040084, 0.033483), new b2Vec2(0.054095, 0.025683), new b2Vec2(0.068617, 0.018872), new b2Vec2(0.083558, 0.013051), new b2Vec2(0.087369, 0.011761), new b2Vec2(0.102790, 0.007261), new b2Vec2(0.118512, 0.003810), new b2Vec2(0.134413, 0.001470), new b2Vec2(0.150435, 0.000210), new b2Vec2(0.166517, 0.000060), new b2Vec2(0.182568, 0.000990), new b2Vec2(0.198530, 0.003060), new b2Vec2(0.214281, 0.006211), new b2Vec2(0.229823, 0.010411), new b2Vec2(0.245005, 0.015692), new b2Vec2(0.259796, 0.022022), new b2Vec2(0.274107, 0.029313), new b2Vec2(0.279598, 0.032523), new b2Vec2(0.282178, 0.034323), new b2Vec2(0.284608, 0.036304), new b2Vec2(0.286889, 0.038434), new b2Vec2(0.289019, 0.040714), new b2Vec2(0.290999, 0.043144), new b2Vec2(0.292799, 0.045695), new b2Vec2(0.294389, 0.048395), new b2Vec2(0.295830, 0.051185), new b2Vec2(0.297030, 0.054035), new b2Vec2(0.298050, 0.057006), new b2Vec2(0.298860, 0.060036), new b2Vec2(0.299460, 0.063096), new b2Vec2(0.299850, 0.066187), new b2Vec2(0.300000, 0.069337), new b2Vec2(0.299940, 0.072457), new b2Vec2(0.299670, 0.075578), new b2Vec2(0.299190, 0.078668), new b2Vec2(0.298500, 0.081698), new b2Vec2(0.297570, 0.084698), new b2Vec2(0.296460, 0.087609), new b2Vec2(0.295140, 0.090459), new b2Vec2(0.293609, 0.093189), new b2Vec2(0.291899, 0.095800), new b2Vec2(0.290039, 0.098290), new b2Vec2(0.287969, 0.100660), new b2Vec2(0.285779, 0.102880), new b2Vec2(0.283408, 0.104920), new b2Vec2(0.280918, 0.106811), new b2Vec2(0.278278, 0.108521), new b2Vec2(0.266877, 0.114731), new b2Vec2(0.252115, 0.121632), new b2Vec2(0.236904, 0.127483), new b2Vec2(0.221332, 0.132223), new b2Vec2(0.205461, 0.135914), new b2Vec2(0.189379, 0.138464), new b2Vec2(0.173147, 0.139904), new b2Vec2(0.156856, 0.140204), new b2Vec2(0.140594, 0.139364), new b2Vec2(0.124422, 0.137414), new b2Vec2(0.108401, 0.134323), new b2Vec2(0.092679, 0.130123), new b2Vec2(0.077258, 0.124842), new b2Vec2(0.062256, 0.118512), new b2Vec2(0.047735, 0.111131), new b2Vec2(0.033753, 0.102760), new b2Vec2(0.020372, 0.093459), new b2Vec2(0.007711, 0.083228));

ptm_ratio = 2500;

contour = [[new b2Vec2(150 / ptm_ratio, 388 / ptm_ratio), new b2Vec2(165 / ptm_ratio, 364 / ptm_ratio), new b2Vec2(208 / ptm_ratio, 330 / ptm_ratio), new b2Vec2(200 / ptm_ratio, 457 / ptm_ratio), new b2Vec2(173 / ptm_ratio, 436 / ptm_ratio), new b2Vec2(159 / ptm_ratio, 423 / ptm_ratio), new b2Vec2(150 / ptm_ratio, 405 / ptm_ratio)], [new b2Vec2(165 / ptm_ratio, 364 / ptm_ratio), new b2Vec2(150 / ptm_ratio, 388 / ptm_ratio), new b2Vec2(155 / ptm_ratio, 375 / ptm_ratio)], [new b2Vec2(208 / ptm_ratio, 330 / ptm_ratio), new b2Vec2(241 / ptm_ratio, 306 / ptm_ratio), new b2Vec2(284 / ptm_ratio, 279 / ptm_ratio), new b2Vec2(338 / ptm_ratio, 251 / ptm_ratio), new b2Vec2(407 / ptm_ratio, 224 / ptm_ratio), new b2Vec2(277 / ptm_ratio, 510 / ptm_ratio), new b2Vec2(234 / ptm_ratio, 482 / ptm_ratio), new b2Vec2(200 / ptm_ratio, 457 / ptm_ratio)], [new b2Vec2(469 / ptm_ratio, 207 / ptm_ratio), new b2Vec2(527 / ptm_ratio, 196 / ptm_ratio), new b2Vec2(483 / ptm_ratio, 589 / ptm_ratio), new b2Vec2(426 / ptm_ratio, 575 / ptm_ratio), new b2Vec2(352 / ptm_ratio, 548 / ptm_ratio), new b2Vec2(314 / ptm_ratio, 530 / ptm_ratio), new b2Vec2(277 / ptm_ratio, 510 / ptm_ratio), new b2Vec2(407 / ptm_ratio, 224 / ptm_ratio)], [new b2Vec2(1031 / ptm_ratio, 354 / ptm_ratio), new b2Vec2(1038 / ptm_ratio, 396 / ptm_ratio), new b2Vec2(1036 / ptm_ratio, 419 / ptm_ratio), new b2Vec2(1021 / ptm_ratio, 463 / ptm_ratio), new b2Vec2(963 / ptm_ratio, 277 / ptm_ratio), new b2Vec2(982 / ptm_ratio, 288 / ptm_ratio), new b2Vec2(1003 / ptm_ratio, 306 / ptm_ratio), new b2Vec2(1021 / ptm_ratio, 330 / ptm_ratio)], [new b2Vec2(1038 / ptm_ratio, 396 / ptm_ratio), new b2Vec2(1031 / ptm_ratio, 354 / ptm_ratio), new b2Vec2(1037 / ptm_ratio, 375 / ptm_ratio)], [new b2Vec2(582 / ptm_ratio, 190 / ptm_ratio), new b2Vec2(642 / ptm_ratio, 189 / ptm_ratio), new b2Vec2(706 / ptm_ratio, 194 / ptm_ratio), new b2Vec2(751 / ptm_ratio, 200 / ptm_ratio), new b2Vec2(589 / ptm_ratio, 603 / ptm_ratio), new b2Vec2(533 / ptm_ratio, 598 / ptm_ratio), new b2Vec2(483 / ptm_ratio, 589 / ptm_ratio), new b2Vec2(527 / ptm_ratio, 196 / ptm_ratio)], [new b2Vec2(971 / ptm_ratio, 512 / ptm_ratio), new b2Vec2(933 / ptm_ratio, 532 / ptm_ratio), new b2Vec2(891 / ptm_ratio, 551 / ptm_ratio), new b2Vec2(811 / ptm_ratio, 579 / ptm_ratio), new b2Vec2(589 / ptm_ratio, 603 / ptm_ratio), new b2Vec2(1021 / ptm_ratio, 463 / ptm_ratio), new b2Vec2(1010 / ptm_ratio, 478 / ptm_ratio), new b2Vec2(995 / ptm_ratio, 495 / ptm_ratio)], [new b2Vec2(794 / ptm_ratio, 210 / ptm_ratio), new b2Vec2(840 / ptm_ratio, 223 / ptm_ratio), new b2Vec2(891 / ptm_ratio, 242 / ptm_ratio), new b2Vec2(929 / ptm_ratio, 259 / ptm_ratio), new b2Vec2(1021 / ptm_ratio, 463 / ptm_ratio), new b2Vec2(589 / ptm_ratio, 603 / ptm_ratio), new b2Vec2(751 / ptm_ratio, 200 / ptm_ratio)], [new b2Vec2(1021 / ptm_ratio, 463 / ptm_ratio), new b2Vec2(1036 / ptm_ratio, 419 / ptm_ratio), new b2Vec2(1031 / ptm_ratio, 441 / ptm_ratio)], [new b2Vec2(1021 / ptm_ratio, 463 / ptm_ratio), new b2Vec2(929 / ptm_ratio, 259 / ptm_ratio), new b2Vec2(963 / ptm_ratio, 277 / ptm_ratio)], [new b2Vec2(811 / ptm_ratio, 579 / ptm_ratio), new b2Vec2(891 / ptm_ratio, 551 / ptm_ratio), new b2Vec2(853 / ptm_ratio, 566 / ptm_ratio)], [new b2Vec2(702 / ptm_ratio, 600 / ptm_ratio), new b2Vec2(665 / ptm_ratio, 603 / ptm_ratio), new b2Vec2(626 / ptm_ratio, 604 / ptm_ratio), new b2Vec2(589 / ptm_ratio, 603 / ptm_ratio), new b2Vec2(811 / ptm_ratio, 579 / ptm_ratio), new b2Vec2(774 / ptm_ratio, 588 / ptm_ratio), new b2Vec2(739 / ptm_ratio, 595 / ptm_ratio)]];

arm1Contour = new Array(new b2Vec2(0.087369, 0.011761), new b2Vec2(0.114761, 0.006781), new b2Vec2(0.142514, 0.009481), new b2Vec2(0.168437, 0.019652), new b2Vec2(0.190579, 0.036574), new b2Vec2(0.204260, 0.053915), new b2Vec2(0.207561, 0.062646), new b2Vec2(0.208341, 0.071947), new b2Vec2(0.206541, 0.081128), new b2Vec2(0.202280, 0.089439), new b2Vec2(0.195920, 0.096250), new b2Vec2(0.187909, 0.101050), new b2Vec2(0.178878, 0.103450), new b2Vec2(0.169547, 0.103300), new b2Vec2(0.160606, 0.100570), new b2Vec2(0.152775, 0.095470), new b2Vec2(0.146625, 0.088419), new b2Vec2(0.139064, 0.076718), new b2Vec2(0.127363, 0.065827), new b2Vec2(0.113111, 0.058566), new b2Vec2(0.097390, 0.055536), new b2Vec2(0.089859, 0.054995), new b2Vec2(0.083978, 0.052895), new b2Vec2(0.078938, 0.049265), new b2Vec2(0.075068, 0.044374), new b2Vec2(0.072697, 0.038644), new b2Vec2(0.072007, 0.032463), new b2Vec2(0.073027, 0.026343), new b2Vec2(0.075698, 0.020702), new b2Vec2(0.079808, 0.016052));

arm2Contour = new Array(new b2Vec2(0.092469, 0.028623), new b2Vec2(0.061926, 0.023882), new b2Vec2(0.033843, 0.011071), new b2Vec2(0.021032, 0.001890), new b2Vec2(0.017462, 0.000420), new b2Vec2(0.013591, 0.000000), new b2Vec2(0.009751, 0.000630), new b2Vec2(0.006271, 0.002310), new b2Vec2(0.003360, 0.004890), new b2Vec2(0.001260, 0.008161), new b2Vec2(0.000150, 0.011881), new b2Vec2(0.000090, 0.015752), new b2Vec2(0.001110, 0.019502), new b2Vec2(0.003120, 0.022832), new b2Vec2(0.026613, 0.041854), new b2Vec2(0.083558, 0.071047), new b2Vec2(0.093099, 0.073417), new b2Vec2(0.099280, 0.072907), new b2Vec2(0.105101, 0.070717), new b2Vec2(0.110081, 0.067027), new b2Vec2(0.113891, 0.062106), new b2Vec2(0.116172, 0.056316), new b2Vec2(0.116772, 0.050135), new b2Vec2(0.115662, 0.044044), new b2Vec2(0.112931, 0.038464), new b2Vec2(0.108761, 0.033843), new b2Vec2(0.103480, 0.030573), new b2Vec2(0.097510, 0.028863));

arm1ContourConvex = [[new b2Vec2(709 / ptm_ratio, 306 / ptm_ratio), new b2Vec2(747 / ptm_ratio, 338 / ptm_ratio), new b2Vec2(569 / ptm_ratio, 577 / ptm_ratio), new b2Vec2(523 / ptm_ratio, 581 / ptm_ratio), new b2Vec2(563 / ptm_ratio, 374 / ptm_ratio), new b2Vec2(583 / ptm_ratio, 342 / ptm_ratio), new b2Vec2(649 / ptm_ratio, 299 / ptm_ratio), new b2Vec2(681 / ptm_ratio, 297 / ptm_ratio)], [new b2Vec2(415 / ptm_ratio, 441 / ptm_ratio), new b2Vec2(439 / ptm_ratio, 438 / ptm_ratio), new b2Vec2(374 / ptm_ratio, 471 / ptm_ratio), new b2Vec2(379 / ptm_ratio, 463 / ptm_ratio), new b2Vec2(398 / ptm_ratio, 447 / ptm_ratio)], [new b2Vec2(767 / ptm_ratio, 393 / ptm_ratio), new b2Vec2(764 / ptm_ratio, 419 / ptm_ratio), new b2Vec2(754 / ptm_ratio, 445 / ptm_ratio), new b2Vec2(715 / ptm_ratio, 495 / ptm_ratio), new b2Vec2(649 / ptm_ratio, 546 / ptm_ratio), new b2Vec2(569 / ptm_ratio, 577 / ptm_ratio), new b2Vec2(747 / ptm_ratio, 338 / ptm_ratio), new b2Vec2(762 / ptm_ratio, 365 / ptm_ratio)], [new b2Vec2(747 / ptm_ratio, 338 / ptm_ratio), new b2Vec2(709 / ptm_ratio, 306 / ptm_ratio), new b2Vec2(730 / ptm_ratio, 319 / ptm_ratio)], [new b2Vec2(374 / ptm_ratio, 544 / ptm_ratio), new b2Vec2(366 / ptm_ratio, 524 / ptm_ratio), new b2Vec2(431 / ptm_ratio, 573 / ptm_ratio), new b2Vec2(413 / ptm_ratio, 570 / ptm_ratio), new b2Vec2(396 / ptm_ratio, 564 / ptm_ratio), new b2Vec2(384 / ptm_ratio, 556 / ptm_ratio)], [new b2Vec2(366 / ptm_ratio, 524 / ptm_ratio), new b2Vec2(363 / ptm_ratio, 509 / ptm_ratio), new b2Vec2(364 / ptm_ratio, 496 / ptm_ratio), new b2Vec2(485 / ptm_ratio, 430 / ptm_ratio), new b2Vec2(527 / ptm_ratio, 410 / ptm_ratio), new b2Vec2(523 / ptm_ratio, 581 / ptm_ratio), new b2Vec2(477 / ptm_ratio, 582 / ptm_ratio), new b2Vec2(431 / ptm_ratio, 573 / ptm_ratio)], [new b2Vec2(485 / ptm_ratio, 430 / ptm_ratio), new b2Vec2(364 / ptm_ratio, 496 / ptm_ratio), new b2Vec2(367 / ptm_ratio, 485 / ptm_ratio), new b2Vec2(374 / ptm_ratio, 471 / ptm_ratio), new b2Vec2(439 / ptm_ratio, 438 / ptm_ratio)], [new b2Vec2(649 / ptm_ratio, 299 / ptm_ratio), new b2Vec2(583 / ptm_ratio, 342 / ptm_ratio), new b2Vec2(606 / ptm_ratio, 318 / ptm_ratio), new b2Vec2(624 / ptm_ratio, 307 / ptm_ratio)], [new b2Vec2(563 / ptm_ratio, 374 / ptm_ratio), new b2Vec2(523 / ptm_ratio, 581 / ptm_ratio), new b2Vec2(527 / ptm_ratio, 410 / ptm_ratio)]];

arm2ContourConvex = [[new b2Vec2(153 / ptm_ratio, 379 / ptm_ratio), new b2Vec2(161 / ptm_ratio, 368 / ptm_ratio), new b2Vec2(178.66667175293 / ptm_ratio, 358 / ptm_ratio), new b2Vec2(230 / ptm_ratio, 374 / ptm_ratio), new b2Vec2(190 / ptm_ratio, 447 / ptm_ratio), new b2Vec2(158 / ptm_ratio, 422 / ptm_ratio), new b2Vec2(149 / ptm_ratio, 400 / ptm_ratio)], [new b2Vec2(211 / ptm_ratio, 359 / ptm_ratio), new b2Vec2(230 / ptm_ratio, 374 / ptm_ratio), new b2Vec2(178.66667175293 / ptm_ratio, 358 / ptm_ratio), new b2Vec2(196 / ptm_ratio, 355.333343505859 / ptm_ratio)], [new b2Vec2(487 / ptm_ratio, 472 / ptm_ratio), new b2Vec2(494 / ptm_ratio, 490 / ptm_ratio), new b2Vec2(496 / ptm_ratio, 509 / ptm_ratio), new b2Vec2(346 / ptm_ratio, 539 / ptm_ratio), new b2Vec2(278 / ptm_ratio, 504 / ptm_ratio), new b2Vec2(458 / ptm_ratio, 447 / ptm_ratio), new b2Vec2(475 / ptm_ratio, 457 / ptm_ratio)], [new b2Vec2(399 / ptm_ratio, 565 / ptm_ratio), new b2Vec2(375 / ptm_ratio, 554 / ptm_ratio), new b2Vec2(346 / ptm_ratio, 539 / ptm_ratio), new b2Vec2(496 / ptm_ratio, 509 / ptm_ratio), new b2Vec2(491 / ptm_ratio, 530 / ptm_ratio), new b2Vec2(464 / ptm_ratio, 563 / ptm_ratio), new b2Vec2(445 / ptm_ratio, 571 / ptm_ratio), new b2Vec2(421 / ptm_ratio, 572 / ptm_ratio)], [new b2Vec2(464 / ptm_ratio, 563 / ptm_ratio), new b2Vec2(491 / ptm_ratio, 530 / ptm_ratio), new b2Vec2(481 / ptm_ratio, 548 / ptm_ratio)], [new b2Vec2(178.66667175293 / ptm_ratio, 358 / ptm_ratio), new b2Vec2(161 / ptm_ratio, 368 / ptm_ratio), new b2Vec2(170 / ptm_ratio, 361 / ptm_ratio)], [new b2Vec2(230 / ptm_ratio, 374 / ptm_ratio), new b2Vec2(247 / ptm_ratio, 386 / ptm_ratio), new b2Vec2(233 / ptm_ratio, 481 / ptm_ratio), new b2Vec2(210 / ptm_ratio, 464 / ptm_ratio), new b2Vec2(190 / ptm_ratio, 447 / ptm_ratio)], [new b2Vec2(268 / ptm_ratio, 396 / ptm_ratio), new b2Vec2(233 / ptm_ratio, 481 / ptm_ratio), new b2Vec2(247 / ptm_ratio, 386 / ptm_ratio)], [new b2Vec2(332 / ptm_ratio, 426 / ptm_ratio), new b2Vec2(278 / ptm_ratio, 504 / ptm_ratio), new b2Vec2(253.33332824707 / ptm_ratio, 492 / ptm_ratio), new b2Vec2(233 / ptm_ratio, 481 / ptm_ratio), new b2Vec2(268 / ptm_ratio, 396 / ptm_ratio)], [new b2Vec2(332 / ptm_ratio, 426 / ptm_ratio), new b2Vec2(458 / ptm_ratio, 447 / ptm_ratio), new b2Vec2(278 / ptm_ratio, 504 / ptm_ratio)], [new b2Vec2(458 / ptm_ratio, 447 / ptm_ratio), new b2Vec2(418 / ptm_ratio, 439 / ptm_ratio), new b2Vec2(438 / ptm_ratio, 440 / ptm_ratio)], [new b2Vec2(391 / ptm_ratio, 435 / ptm_ratio), new b2Vec2(332 / ptm_ratio, 426 / ptm_ratio), new b2Vec2(364 / ptm_ratio, 430 / ptm_ratio)], [new b2Vec2(458 / ptm_ratio, 447 / ptm_ratio), new b2Vec2(391 / ptm_ratio, 435 / ptm_ratio), new b2Vec2(418 / ptm_ratio, 439 / ptm_ratio)]];
