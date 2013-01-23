// Generated by CoffeeScript 1.3.3
var arm1Center, arm1ContourConvex, arm1JointAnchor, arm2Center, arm2ContourConvex, arm2JointAnchor, b2Vec2, contour, contourCenter, contour_original_high_detail, contour_original_low_detail, head, ptm_ratio, ptm_ratio2;

b2Vec2 = Box2D.Common.Math.b2Vec2;

ptm_ratio = 1500;

contour = [[new b2Vec2(276 / ptm_ratio, 501 / ptm_ratio), new b2Vec2(277 / ptm_ratio, 487 / ptm_ratio), new b2Vec2(293 / ptm_ratio, 461 / ptm_ratio), new b2Vec2(334 / ptm_ratio, 429 / ptm_ratio), new b2Vec2(341 / ptm_ratio, 568 / ptm_ratio), new b2Vec2(293 / ptm_ratio, 529 / ptm_ratio), new b2Vec2(280 / ptm_ratio, 515 / ptm_ratio)], [new b2Vec2(293 / ptm_ratio, 461 / ptm_ratio), new b2Vec2(277 / ptm_ratio, 487 / ptm_ratio), new b2Vec2(282 / ptm_ratio, 474 / ptm_ratio)], [new b2Vec2(368 / ptm_ratio, 405 / ptm_ratio), new b2Vec2(411 / ptm_ratio, 378 / ptm_ratio), new b2Vec2(450 / ptm_ratio, 358 / ptm_ratio), new b2Vec2(420 / ptm_ratio, 618 / ptm_ratio), new b2Vec2(396 / ptm_ratio, 605 / ptm_ratio), new b2Vec2(341 / ptm_ratio, 568 / ptm_ratio), new b2Vec2(334 / ptm_ratio, 429 / ptm_ratio)], [new b2Vec2(1146 / ptm_ratio, 429 / ptm_ratio), new b2Vec2(1162 / ptm_ratio, 473 / ptm_ratio), new b2Vec2(1164 / ptm_ratio, 495 / ptm_ratio), new b2Vec2(1025 / ptm_ratio, 647 / ptm_ratio), new b2Vec2(1059 / ptm_ratio, 360 / ptm_ratio), new b2Vec2(1086 / ptm_ratio, 374 / ptm_ratio), new b2Vec2(1111 / ptm_ratio, 390 / ptm_ratio), new b2Vec2(1130 / ptm_ratio, 407 / ptm_ratio)], [new b2Vec2(1162 / ptm_ratio, 473 / ptm_ratio), new b2Vec2(1146 / ptm_ratio, 429 / ptm_ratio), new b2Vec2(1156 / ptm_ratio, 450 / ptm_ratio)], [new b2Vec2(1118 / ptm_ratio, 595 / ptm_ratio), new b2Vec2(1096 / ptm_ratio, 612 / ptm_ratio), new b2Vec2(1060 / ptm_ratio, 631 / ptm_ratio), new b2Vec2(1025 / ptm_ratio, 647 / ptm_ratio), new b2Vec2(1164 / ptm_ratio, 495 / ptm_ratio), new b2Vec2(1156 / ptm_ratio, 540 / ptm_ratio), new b2Vec2(1147 / ptm_ratio, 560 / ptm_ratio), new b2Vec2(1136 / ptm_ratio, 577 / ptm_ratio)], [new b2Vec2(1156 / ptm_ratio, 540 / ptm_ratio), new b2Vec2(1164 / ptm_ratio, 495 / ptm_ratio), new b2Vec2(1162 / ptm_ratio, 518 / ptm_ratio)], [new b2Vec2(450 / ptm_ratio, 358 / ptm_ratio), new b2Vec2(468 / ptm_ratio, 349 / ptm_ratio), new b2Vec2(528 / ptm_ratio, 666 / ptm_ratio), new b2Vec2(506 / ptm_ratio, 658 / ptm_ratio), new b2Vec2(441 / ptm_ratio, 629 / ptm_ratio), new b2Vec2(420 / ptm_ratio, 618 / ptm_ratio)], [new b2Vec2(1025 / ptm_ratio, 647 / ptm_ratio), new b2Vec2(998 / ptm_ratio, 658 / ptm_ratio), new b2Vec2(970 / ptm_ratio, 324 / ptm_ratio), new b2Vec2(994 / ptm_ratio, 332 / ptm_ratio), new b2Vec2(1026 / ptm_ratio, 345 / ptm_ratio), new b2Vec2(1059 / ptm_ratio, 360 / ptm_ratio)], [new b2Vec2(861 / ptm_ratio, 694 / ptm_ratio), new b2Vec2(878 / ptm_ratio, 691 / ptm_ratio), new b2Vec2(870 / ptm_ratio, 693 / ptm_ratio)], [new b2Vec2(503 / ptm_ratio, 334 / ptm_ratio), new b2Vec2(553 / ptm_ratio, 317 / ptm_ratio), new b2Vec2(546 / ptm_ratio, 672 / ptm_ratio), new b2Vec2(528 / ptm_ratio, 666 / ptm_ratio), new b2Vec2(468 / ptm_ratio, 349 / ptm_ratio)], [new b2Vec2(998 / ptm_ratio, 658 / ptm_ratio), new b2Vec2(981 / ptm_ratio, 664 / ptm_ratio), new b2Vec2(965 / ptm_ratio, 669 / ptm_ratio), new b2Vec2(932 / ptm_ratio, 312 / ptm_ratio), new b2Vec2(950 / ptm_ratio, 317 / ptm_ratio), new b2Vec2(970 / ptm_ratio, 324 / ptm_ratio)], [new b2Vec2(623 / ptm_ratio, 300 / ptm_ratio), new b2Vec2(546 / ptm_ratio, 672 / ptm_ratio), new b2Vec2(587 / ptm_ratio, 308 / ptm_ratio), new b2Vec2(605 / ptm_ratio, 303 / ptm_ratio)], [new b2Vec2(786 / ptm_ratio, 289 / ptm_ratio), new b2Vec2(928 / ptm_ratio, 680 / ptm_ratio), new b2Vec2(731 / ptm_ratio, 288 / ptm_ratio), new b2Vec2(749 / ptm_ratio, 287 / ptm_ratio)], [new b2Vec2(947 / ptm_ratio, 675 / ptm_ratio), new b2Vec2(928 / ptm_ratio, 680 / ptm_ratio), new b2Vec2(879 / ptm_ratio, 300 / ptm_ratio), new b2Vec2(897 / ptm_ratio, 303 / ptm_ratio), new b2Vec2(915 / ptm_ratio, 307 / ptm_ratio), new b2Vec2(932 / ptm_ratio, 312 / ptm_ratio), new b2Vec2(965 / ptm_ratio, 669 / ptm_ratio)], [new b2Vec2(570 / ptm_ratio, 312 / ptm_ratio), new b2Vec2(587 / ptm_ratio, 308 / ptm_ratio), new b2Vec2(546 / ptm_ratio, 672 / ptm_ratio), new b2Vec2(553 / ptm_ratio, 317 / ptm_ratio)], [new b2Vec2(928 / ptm_ratio, 680 / ptm_ratio), new b2Vec2(786 / ptm_ratio, 289 / ptm_ratio), new b2Vec2(843 / ptm_ratio, 294 / ptm_ratio), new b2Vec2(879 / ptm_ratio, 300 / ptm_ratio)], [new b2Vec2(576 / ptm_ratio, 680 / ptm_ratio), new b2Vec2(639 / ptm_ratio, 297 / ptm_ratio), new b2Vec2(671 / ptm_ratio, 698 / ptm_ratio), new b2Vec2(640 / ptm_ratio, 694 / ptm_ratio), new b2Vec2(593 / ptm_ratio, 685 / ptm_ratio)], [new b2Vec2(782 / ptm_ratio, 702 / ptm_ratio), new b2Vec2(639 / ptm_ratio, 297 / ptm_ratio), new b2Vec2(653 / ptm_ratio, 295 / ptm_ratio), new b2Vec2(807 / ptm_ratio, 701 / ptm_ratio), new b2Vec2(797 / ptm_ratio, 702 / ptm_ratio)], [new b2Vec2(639 / ptm_ratio, 297 / ptm_ratio), new b2Vec2(576 / ptm_ratio, 680 / ptm_ratio), new b2Vec2(546 / ptm_ratio, 672 / ptm_ratio), new b2Vec2(623 / ptm_ratio, 300 / ptm_ratio)], [new b2Vec2(697 / ptm_ratio, 290 / ptm_ratio), new b2Vec2(928 / ptm_ratio, 680 / ptm_ratio), new b2Vec2(888 / ptm_ratio, 689 / ptm_ratio), new b2Vec2(807 / ptm_ratio, 701 / ptm_ratio), new b2Vec2(666 / ptm_ratio, 293 / ptm_ratio), new b2Vec2(680 / ptm_ratio, 291 / ptm_ratio)], [new b2Vec2(639 / ptm_ratio, 297 / ptm_ratio), new b2Vec2(704 / ptm_ratio, 701 / ptm_ratio), new b2Vec2(687 / ptm_ratio, 700 / ptm_ratio), new b2Vec2(671 / ptm_ratio, 698 / ptm_ratio)], [new b2Vec2(639 / ptm_ratio, 297 / ptm_ratio), new b2Vec2(782 / ptm_ratio, 702 / ptm_ratio), new b2Vec2(767 / ptm_ratio, 703 / ptm_ratio), new b2Vec2(735 / ptm_ratio, 703 / ptm_ratio), new b2Vec2(704 / ptm_ratio, 701 / ptm_ratio)], [new b2Vec2(928 / ptm_ratio, 680 / ptm_ratio), new b2Vec2(697 / ptm_ratio, 290 / ptm_ratio), new b2Vec2(731 / ptm_ratio, 288 / ptm_ratio)], [new b2Vec2(666 / ptm_ratio, 293 / ptm_ratio), new b2Vec2(807 / ptm_ratio, 701 / ptm_ratio), new b2Vec2(653 / ptm_ratio, 295 / ptm_ratio)], [new b2Vec2(907 / ptm_ratio, 685 / ptm_ratio), new b2Vec2(928 / ptm_ratio, 680 / ptm_ratio), new b2Vec2(916 / ptm_ratio, 683 / ptm_ratio)], [new b2Vec2(888 / ptm_ratio, 689 / ptm_ratio), new b2Vec2(907 / ptm_ratio, 685 / ptm_ratio), new b2Vec2(898 / ptm_ratio, 687 / ptm_ratio)], [new b2Vec2(854 / ptm_ratio, 695 / ptm_ratio), new b2Vec2(814 / ptm_ratio, 700 / ptm_ratio), new b2Vec2(888 / ptm_ratio, 689 / ptm_ratio)], [new b2Vec2(832 / ptm_ratio, 698 / ptm_ratio), new b2Vec2(847 / ptm_ratio, 696 / ptm_ratio), new b2Vec2(840 / ptm_ratio, 697 / ptm_ratio)]];

contourCenter = new b2Vec2(820 / ptm_ratio, 502 / ptm_ratio);

arm1ContourConvex = [[new b2Vec2(885 / ptm_ratio, 536 / ptm_ratio), new b2Vec2(851 / ptm_ratio, 583 / ptm_ratio), new b2Vec2(814 / ptm_ratio, 619 / ptm_ratio), new b2Vec2(691 / ptm_ratio, 473 / ptm_ratio), new b2Vec2(804 / ptm_ratio, 397 / ptm_ratio), new b2Vec2(844 / ptm_ratio, 410 / ptm_ratio), new b2Vec2(878 / ptm_ratio, 443 / ptm_ratio), new b2Vec2(893 / ptm_ratio, 488 / ptm_ratio)], [new b2Vec2(804 / ptm_ratio, 397 / ptm_ratio), new b2Vec2(691 / ptm_ratio, 473 / ptm_ratio), new b2Vec2(720 / ptm_ratio, 429 / ptm_ratio), new b2Vec2(760 / ptm_ratio, 402 / ptm_ratio)], [new b2Vec2(814 / ptm_ratio, 619 / ptm_ratio), new b2Vec2(776 / ptm_ratio, 644 / ptm_ratio), new b2Vec2(655 / ptm_ratio, 508 / ptm_ratio), new b2Vec2(691 / ptm_ratio, 473 / ptm_ratio)], [new b2Vec2(776 / ptm_ratio, 644 / ptm_ratio), new b2Vec2(731 / ptm_ratio, 665 / ptm_ratio), new b2Vec2(681 / ptm_ratio, 679 / ptm_ratio), new b2Vec2(628 / ptm_ratio, 684 / ptm_ratio), new b2Vec2(602 / ptm_ratio, 534 / ptm_ratio), new b2Vec2(655 / ptm_ratio, 508 / ptm_ratio)], [new b2Vec2(628 / ptm_ratio, 684 / ptm_ratio), new b2Vec2(593 / ptm_ratio, 681 / ptm_ratio), new b2Vec2(559 / ptm_ratio, 675 / ptm_ratio), new b2Vec2(531 / ptm_ratio, 667 / ptm_ratio), new b2Vec2(510 / ptm_ratio, 654 / ptm_ratio), new b2Vec2(526 / ptm_ratio, 546 / ptm_ratio), new b2Vec2(548 / ptm_ratio, 539 / ptm_ratio), new b2Vec2(602 / ptm_ratio, 534 / ptm_ratio)], [new b2Vec2(526 / ptm_ratio, 546 / ptm_ratio), new b2Vec2(510 / ptm_ratio, 654 / ptm_ratio), new b2Vec2(498 / ptm_ratio, 639 / ptm_ratio), new b2Vec2(490 / ptm_ratio, 618 / ptm_ratio), new b2Vec2(489 / ptm_ratio, 598 / ptm_ratio), new b2Vec2(496 / ptm_ratio, 575 / ptm_ratio), new b2Vec2(509 / ptm_ratio, 557 / ptm_ratio)]];

arm1JointAnchor = new b2Vec2(794 / ptm_ratio, 496 / ptm_ratio);

arm1Center = new b2Vec2(743 / ptm_ratio, 535 / ptm_ratio);

arm2ContourConvex = [[new b2Vec2(278 / ptm_ratio, 485 / ptm_ratio), new b2Vec2(294 / ptm_ratio, 461 / ptm_ratio), new b2Vec2(338 / ptm_ratio, 459 / ptm_ratio), new b2Vec2(382 / ptm_ratio, 491 / ptm_ratio), new b2Vec2(358 / ptm_ratio, 580 / ptm_ratio), new b2Vec2(319 / ptm_ratio, 551 / ptm_ratio), new b2Vec2(284 / ptm_ratio, 521 / ptm_ratio), new b2Vec2(276 / ptm_ratio, 501 / ptm_ratio)], [new b2Vec2(294 / ptm_ratio, 461 / ptm_ratio), new b2Vec2(278 / ptm_ratio, 485 / ptm_ratio), new b2Vec2(283 / ptm_ratio, 472 / ptm_ratio)], [new b2Vec2(543 / ptm_ratio, 671 / ptm_ratio), new b2Vec2(499 / ptm_ratio, 655 / ptm_ratio), new b2Vec2(517 / ptm_ratio, 537 / ptm_ratio), new b2Vec2(545 / ptm_ratio, 539 / ptm_ratio), new b2Vec2(584 / ptm_ratio, 545 / ptm_ratio), new b2Vec2(622 / ptm_ratio, 608 / ptm_ratio), new b2Vec2(590 / ptm_ratio, 662 / ptm_ratio), new b2Vec2(570 / ptm_ratio, 671 / ptm_ratio)], [new b2Vec2(338 / ptm_ratio, 459 / ptm_ratio), new b2Vec2(294 / ptm_ratio, 461 / ptm_ratio), new b2Vec2(308 / ptm_ratio, 455 / ptm_ratio), new b2Vec2(322 / ptm_ratio, 454 / ptm_ratio)], [new b2Vec2(446 / ptm_ratio, 521 / ptm_ratio), new b2Vec2(402 / ptm_ratio, 608 / ptm_ratio), new b2Vec2(358 / ptm_ratio, 580 / ptm_ratio), new b2Vec2(382 / ptm_ratio, 491 / ptm_ratio)], [new b2Vec2(517 / ptm_ratio, 537 / ptm_ratio), new b2Vec2(499 / ptm_ratio, 655 / ptm_ratio), new b2Vec2(468 / ptm_ratio, 642 / ptm_ratio), new b2Vec2(402 / ptm_ratio, 608 / ptm_ratio), new b2Vec2(446 / ptm_ratio, 521 / ptm_ratio)], [new b2Vec2(606 / ptm_ratio, 649 / ptm_ratio), new b2Vec2(590 / ptm_ratio, 662 / ptm_ratio), new b2Vec2(622 / ptm_ratio, 608 / ptm_ratio), new b2Vec2(618 / ptm_ratio, 629 / ptm_ratio)], [new b2Vec2(613 / ptm_ratio, 571 / ptm_ratio), new b2Vec2(620 / ptm_ratio, 589 / ptm_ratio), new b2Vec2(622 / ptm_ratio, 608 / ptm_ratio), new b2Vec2(584 / ptm_ratio, 545 / ptm_ratio), new b2Vec2(601 / ptm_ratio, 556 / ptm_ratio)], [new b2Vec2(584 / ptm_ratio, 545 / ptm_ratio), new b2Vec2(545 / ptm_ratio, 539 / ptm_ratio), new b2Vec2(565 / ptm_ratio, 539 / ptm_ratio)]];

arm2JointAnchor = new b2Vec2(554 / ptm_ratio, 605 / ptm_ratio);

arm2Center = new b2Vec2(447 / ptm_ratio, 568 / ptm_ratio);

head = [new b2Vec2(1030.545 / ptm_ratio, 496.364 / ptm_ratio), 117.004 / ptm_ratio];

ptm_ratio2 = 16890;

contour_original_high_detail = [new b2Vec2(151 / ptm_ratio2, 2681 / ptm_ratio2), new b2Vec2(82 / ptm_ratio2, 2601 / ptm_ratio2), new b2Vec2(0 / ptm_ratio2, 2359 / ptm_ratio2), new b2Vec2(8 / ptm_ratio2, 2245 / ptm_ratio2), new b2Vec2(85 / ptm_ratio2, 2068 / ptm_ratio2), new b2Vec2(155 / ptm_ratio2, 1989 / ptm_ratio2), new b2Vec2(357 / ptm_ratio2, 1814 / ptm_ratio2), new b2Vec2(460 / ptm_ratio2, 1729 / ptm_ratio2), new b2Vec2(671 / ptm_ratio2, 1565 / ptm_ratio2), new b2Vec2(779 / ptm_ratio2, 1485 / ptm_ratio2), new b2Vec2(998 / ptm_ratio2, 1332 / ptm_ratio2), new b2Vec2(1109 / ptm_ratio2, 1258 / ptm_ratio2), new b2Vec2(1336 / ptm_ratio2, 1116 / ptm_ratio2), new b2Vec2(1451 / ptm_ratio2, 1048 / ptm_ratio2), new b2Vec2(1685 / ptm_ratio2, 918 / ptm_ratio2), new b2Vec2(1803 / ptm_ratio2, 856 / ptm_ratio2), new b2Vec2(2043 / ptm_ratio2, 739 / ptm_ratio2), new b2Vec2(2165 / ptm_ratio2, 683 / ptm_ratio2), new b2Vec2(2410 / ptm_ratio2, 577 / ptm_ratio2), new b2Vec2(2535 / ptm_ratio2, 528 / ptm_ratio2), new b2Vec2(2785 / ptm_ratio2, 435 / ptm_ratio2), new b2Vec2(2912 / ptm_ratio2, 392 / ptm_ratio2), new b2Vec2(3168 / ptm_ratio2, 313 / ptm_ratio2), new b2Vec2(3297 / ptm_ratio2, 276 / ptm_ratio2), new b2Vec2(3556 / ptm_ratio2, 210 / ptm_ratio2), new b2Vec2(3687 / ptm_ratio2, 180 / ptm_ratio2), new b2Vec2(3950 / ptm_ratio2, 127 / ptm_ratio2), new b2Vec2(4082 / ptm_ratio2, 104 / ptm_ratio2), new b2Vec2(4347 / ptm_ratio2, 65 / ptm_ratio2), new b2Vec2(4480 / ptm_ratio2, 49 / ptm_ratio2), new b2Vec2(4747 / ptm_ratio2, 23 / ptm_ratio2), new b2Vec2(4881 / ptm_ratio2, 14 / ptm_ratio2), new b2Vec2(5148 / ptm_ratio2, 2 / ptm_ratio2), new b2Vec2(5282 / ptm_ratio2, 0 / ptm_ratio2), new b2Vec2(5550 / ptm_ratio2, 2 / ptm_ratio2), new b2Vec2(5684 / ptm_ratio2, 6 / ptm_ratio2), new b2Vec2(5952 / ptm_ratio2, 22 / ptm_ratio2), new b2Vec2(6085 / ptm_ratio2, 33 / ptm_ratio2), new b2Vec2(6352 / ptm_ratio2, 63 / ptm_ratio2), new b2Vec2(6484 / ptm_ratio2, 81 / ptm_ratio2), new b2Vec2(6749 / ptm_ratio2, 125 / ptm_ratio2), new b2Vec2(6881 / ptm_ratio2, 150 / ptm_ratio2), new b2Vec2(7142 / ptm_ratio2, 207 / ptm_ratio2), new b2Vec2(7273 / ptm_ratio2, 239 / ptm_ratio2), new b2Vec2(7402 / ptm_ratio2, 273 / ptm_ratio2), new b2Vec2(7660 / ptm_ratio2, 347 / ptm_ratio2), new b2Vec2(7787 / ptm_ratio2, 388 / ptm_ratio2), new b2Vec2(8040 / ptm_ratio2, 476 / ptm_ratio2), new b2Vec2(8166 / ptm_ratio2, 523 / ptm_ratio2), new b2Vec2(8414 / ptm_ratio2, 624 / ptm_ratio2), new b2Vec2(8537 / ptm_ratio2, 678 / ptm_ratio2), new b2Vec2(8780 / ptm_ratio2, 792 / ptm_ratio2), new b2Vec2(8900 / ptm_ratio2, 851 / ptm_ratio2), new b2Vec2(9018 / ptm_ratio2, 913 / ptm_ratio2), new b2Vec2(9253 / ptm_ratio2, 1043 / ptm_ratio2), new b2Vec2(9363 / ptm_ratio2, 1113 / ptm_ratio2), new b2Vec2(9525 / ptm_ratio2, 1244 / ptm_ratio2), new b2Vec2(9616 / ptm_ratio2, 1337 / ptm_ratio2), new b2Vec2(9729 / ptm_ratio2, 1480 / ptm_ratio2), new b2Vec2(9786 / ptm_ratio2, 1568 / ptm_ratio2), new b2Vec2(9891 / ptm_ratio2, 1777 / ptm_ratio2), new b2Vec2(9900 / ptm_ratio2, 1801 / ptm_ratio2), new b2Vec2(9934 / ptm_ratio2, 1900 / ptm_ratio2), new b2Vec2(9949 / ptm_ratio2, 1950 / ptm_ratio2), new b2Vec2(9961 / ptm_ratio2, 2001 / ptm_ratio2), new b2Vec2(9972 / ptm_ratio2, 2052 / ptm_ratio2), new b2Vec2(9999 / ptm_ratio2, 2363 / ptm_ratio2), new b2Vec2(9977 / ptm_ratio2, 2596 / ptm_ratio2), new b2Vec2(9955 / ptm_ratio2, 2698 / ptm_ratio2), new b2Vec2(9942 / ptm_ratio2, 2748 / ptm_ratio2), new b2Vec2(9926 / ptm_ratio2, 2798 / ptm_ratio2), new b2Vec2(9909 / ptm_ratio2, 2847 / ptm_ratio2), new b2Vec2(9837 / ptm_ratio2, 3015 / ptm_ratio2), new b2Vec2(9799 / ptm_ratio2, 3083 / ptm_ratio2), new b2Vec2(9773 / ptm_ratio2, 3128 / ptm_ratio2), new b2Vec2(9759 / ptm_ratio2, 3150 / ptm_ratio2), new b2Vec2(9714 / ptm_ratio2, 3214 / ptm_ratio2), new b2Vec2(9699 / ptm_ratio2, 3235 / ptm_ratio2), new b2Vec2(9667 / ptm_ratio2, 3276 / ptm_ratio2), new b2Vec2(9633 / ptm_ratio2, 3316 / ptm_ratio2), new b2Vec2(9580 / ptm_ratio2, 3374 / ptm_ratio2), new b2Vec2(9505 / ptm_ratio2, 3446 / ptm_ratio2), new b2Vec2(9446 / ptm_ratio2, 3497 / ptm_ratio2), new b2Vec2(9405 / ptm_ratio2, 3529 / ptm_ratio2), new b2Vec2(9363 / ptm_ratio2, 3560 / ptm_ratio2), new b2Vec2(9319 / ptm_ratio2, 3589 / ptm_ratio2), new b2Vec2(9135 / ptm_ratio2, 3697 / ptm_ratio2), new b2Vec2(9015 / ptm_ratio2, 3762 / ptm_ratio2), new b2Vec2(8773 / ptm_ratio2, 3885 / ptm_ratio2), new b2Vec2(8651 / ptm_ratio2, 3943 / ptm_ratio2), new b2Vec2(8403 / ptm_ratio2, 4054 / ptm_ratio2), new b2Vec2(8277 / ptm_ratio2, 4106 / ptm_ratio2), new b2Vec2(8024 / ptm_ratio2, 4203 / ptm_ratio2), new b2Vec2(7896 / ptm_ratio2, 4249 / ptm_ratio2), new b2Vec2(7638 / ptm_ratio2, 4333 / ptm_ratio2), new b2Vec2(7508 / ptm_ratio2, 4371 / ptm_ratio2), new b2Vec2(7245 / ptm_ratio2, 4442 / ptm_ratio2), new b2Vec2(7113 / ptm_ratio2, 4473 / ptm_ratio2), new b2Vec2(6848 / ptm_ratio2, 4530 / ptm_ratio2), new b2Vec2(6714 / ptm_ratio2, 4555 / ptm_ratio2), new b2Vec2(6446 / ptm_ratio2, 4597 / ptm_ratio2), new b2Vec2(6312 / ptm_ratio2, 4615 / ptm_ratio2), new b2Vec2(6042 / ptm_ratio2, 4644 / ptm_ratio2), new b2Vec2(5906 / ptm_ratio2, 4655 / ptm_ratio2), new b2Vec2(5635 / ptm_ratio2, 4669 / ptm_ratio2), new b2Vec2(5499 / ptm_ratio2, 4673 / ptm_ratio2), new b2Vec2(5228 / ptm_ratio2, 4673 / ptm_ratio2), new b2Vec2(5092 / ptm_ratio2, 4670 / ptm_ratio2), new b2Vec2(4821 / ptm_ratio2, 4656 / ptm_ratio2), new b2Vec2(4686 / ptm_ratio2, 4645 / ptm_ratio2), new b2Vec2(4415 / ptm_ratio2, 4617 / ptm_ratio2), new b2Vec2(4281 / ptm_ratio2, 4599 / ptm_ratio2), new b2Vec2(4013 / ptm_ratio2, 4557 / ptm_ratio2), new b2Vec2(3879 / ptm_ratio2, 4533 / ptm_ratio2), new b2Vec2(3613 / ptm_ratio2, 4477 / ptm_ratio2), new b2Vec2(3481 / ptm_ratio2, 4445 / ptm_ratio2), new b2Vec2(3219 / ptm_ratio2, 4375 / ptm_ratio2), new b2Vec2(3089 / ptm_ratio2, 4337 / ptm_ratio2), new b2Vec2(2830 / ptm_ratio2, 4253 / ptm_ratio2), new b2Vec2(2702 / ptm_ratio2, 4208 / ptm_ratio2), new b2Vec2(2449 / ptm_ratio2, 4111 / ptm_ratio2), new b2Vec2(2323 / ptm_ratio2, 4060 / ptm_ratio2), new b2Vec2(2075 / ptm_ratio2, 3950 / ptm_ratio2), new b2Vec2(1952 / ptm_ratio2, 3891 / ptm_ratio2), new b2Vec2(1710 / ptm_ratio2, 3768 / ptm_ratio2), new b2Vec2(1591 / ptm_ratio2, 3704 / ptm_ratio2), new b2Vec2(1355 / ptm_ratio2, 3569 / ptm_ratio2), new b2Vec2(1239 / ptm_ratio2, 3498 / ptm_ratio2), new b2Vec2(1011 / ptm_ratio2, 3350 / ptm_ratio2), new b2Vec2(899 / ptm_ratio2, 3274 / ptm_ratio2), new b2Vec2(789 / ptm_ratio2, 3195 / ptm_ratio2), new b2Vec2(679 / ptm_ratio2, 3115 / ptm_ratio2), new b2Vec2(571 / ptm_ratio2, 3032 / ptm_ratio2), new b2Vec2(465 / ptm_ratio2, 2948 / ptm_ratio2), new b2Vec2(360 / ptm_ratio2, 2862 / ptm_ratio2), new b2Vec2(257 / ptm_ratio2, 2774 / ptm_ratio2)];

contour_original_low_detail = [new b2Vec2(151 / ptm_ratio2, 2681 / ptm_ratio2), new b2Vec2(82 / ptm_ratio2, 2601 / ptm_ratio2), new b2Vec2(0 / ptm_ratio2, 2359 / ptm_ratio2), new b2Vec2(8 / ptm_ratio2, 2245 / ptm_ratio2), new b2Vec2(85 / ptm_ratio2, 2068 / ptm_ratio2), new b2Vec2(155 / ptm_ratio2, 1989 / ptm_ratio2), new b2Vec2(460 / ptm_ratio2, 1729 / ptm_ratio2), new b2Vec2(779 / ptm_ratio2, 1485 / ptm_ratio2), new b2Vec2(1336 / ptm_ratio2, 1116 / ptm_ratio2), new b2Vec2(1451 / ptm_ratio2, 1048 / ptm_ratio2), new b2Vec2(2043 / ptm_ratio2, 739 / ptm_ratio2), new b2Vec2(2410 / ptm_ratio2, 577 / ptm_ratio2), new b2Vec2(2912 / ptm_ratio2, 392 / ptm_ratio2), new b2Vec2(3168 / ptm_ratio2, 313 / ptm_ratio2), new b2Vec2(3687 / ptm_ratio2, 180 / ptm_ratio2), new b2Vec2(4082 / ptm_ratio2, 104 / ptm_ratio2), new b2Vec2(4747 / ptm_ratio2, 23 / ptm_ratio2), new b2Vec2(4881 / ptm_ratio2, 14 / ptm_ratio2), new b2Vec2(5148 / ptm_ratio2, 2 / ptm_ratio2), new b2Vec2(5282 / ptm_ratio2, 0 / ptm_ratio2), new b2Vec2(5416 / ptm_ratio2, 0 / ptm_ratio2), new b2Vec2(5550 / ptm_ratio2, 2 / ptm_ratio2), new b2Vec2(5684 / ptm_ratio2, 6 / ptm_ratio2), new b2Vec2(5818 / ptm_ratio2, 13 / ptm_ratio2), new b2Vec2(5952 / ptm_ratio2, 22 / ptm_ratio2), new b2Vec2(6085 / ptm_ratio2, 33 / ptm_ratio2), new b2Vec2(6219 / ptm_ratio2, 47 / ptm_ratio2), new b2Vec2(6352 / ptm_ratio2, 63 / ptm_ratio2), new b2Vec2(6484 / ptm_ratio2, 81 / ptm_ratio2), new b2Vec2(6617 / ptm_ratio2, 102 / ptm_ratio2), new b2Vec2(6749 / ptm_ratio2, 125 / ptm_ratio2), new b2Vec2(6881 / ptm_ratio2, 150 / ptm_ratio2), new b2Vec2(7012 / ptm_ratio2, 177 / ptm_ratio2), new b2Vec2(7142 / ptm_ratio2, 207 / ptm_ratio2), new b2Vec2(7273 / ptm_ratio2, 239 / ptm_ratio2), new b2Vec2(7402 / ptm_ratio2, 273 / ptm_ratio2), new b2Vec2(7531 / ptm_ratio2, 309 / ptm_ratio2), new b2Vec2(7660 / ptm_ratio2, 347 / ptm_ratio2), new b2Vec2(7787 / ptm_ratio2, 388 / ptm_ratio2), new b2Vec2(7914 / ptm_ratio2, 431 / ptm_ratio2), new b2Vec2(8040 / ptm_ratio2, 476 / ptm_ratio2), new b2Vec2(8166 / ptm_ratio2, 523 / ptm_ratio2), new b2Vec2(8290 / ptm_ratio2, 573 / ptm_ratio2), new b2Vec2(8414 / ptm_ratio2, 624 / ptm_ratio2), new b2Vec2(8537 / ptm_ratio2, 678 / ptm_ratio2), new b2Vec2(8659 / ptm_ratio2, 734 / ptm_ratio2), new b2Vec2(8780 / ptm_ratio2, 792 / ptm_ratio2), new b2Vec2(8900 / ptm_ratio2, 851 / ptm_ratio2), new b2Vec2(9018 / ptm_ratio2, 913 / ptm_ratio2), new b2Vec2(9253 / ptm_ratio2, 1043 / ptm_ratio2), new b2Vec2(9616 / ptm_ratio2, 1337 / ptm_ratio2), new b2Vec2(9729 / ptm_ratio2, 1480 / ptm_ratio2), new b2Vec2(9900 / ptm_ratio2, 1801 / ptm_ratio2), new b2Vec2(9949 / ptm_ratio2, 1950 / ptm_ratio2), new b2Vec2(9999 / ptm_ratio2, 2363 / ptm_ratio2), new b2Vec2(9977 / ptm_ratio2, 2596 / ptm_ratio2), new b2Vec2(9837 / ptm_ratio2, 3015 / ptm_ratio2), new b2Vec2(9759 / ptm_ratio2, 3150 / ptm_ratio2), new b2Vec2(9714 / ptm_ratio2, 3214 / ptm_ratio2), new b2Vec2(9505 / ptm_ratio2, 3446 / ptm_ratio2), new b2Vec2(9363 / ptm_ratio2, 3560 / ptm_ratio2), new b2Vec2(8773 / ptm_ratio2, 3885 / ptm_ratio2), new b2Vec2(8403 / ptm_ratio2, 4054 / ptm_ratio2), new b2Vec2(7896 / ptm_ratio2, 4249 / ptm_ratio2), new b2Vec2(7638 / ptm_ratio2, 4333 / ptm_ratio2), new b2Vec2(7113 / ptm_ratio2, 4473 / ptm_ratio2), new b2Vec2(6714 / ptm_ratio2, 4555 / ptm_ratio2), new b2Vec2(6042 / ptm_ratio2, 4644 / ptm_ratio2), new b2Vec2(5906 / ptm_ratio2, 4655 / ptm_ratio2), new b2Vec2(5228 / ptm_ratio2, 4673 / ptm_ratio2), new b2Vec2(4821 / ptm_ratio2, 4656 / ptm_ratio2), new b2Vec2(4281 / ptm_ratio2, 4599 / ptm_ratio2), new b2Vec2(4013 / ptm_ratio2, 4557 / ptm_ratio2), new b2Vec2(3481 / ptm_ratio2, 4445 / ptm_ratio2), new b2Vec2(3089 / ptm_ratio2, 4337 / ptm_ratio2), new b2Vec2(2449 / ptm_ratio2, 4111 / ptm_ratio2), new b2Vec2(2323 / ptm_ratio2, 4060 / ptm_ratio2), new b2Vec2(1710 / ptm_ratio2, 3768 / ptm_ratio2), new b2Vec2(1355 / ptm_ratio2, 3569 / ptm_ratio2), new b2Vec2(899 / ptm_ratio2, 3274 / ptm_ratio2), new b2Vec2(679 / ptm_ratio2, 3115 / ptm_ratio2), new b2Vec2(360 / ptm_ratio2, 2862 / ptm_ratio2), new b2Vec2(257 / ptm_ratio2, 2774 / ptm_ratio2)];
