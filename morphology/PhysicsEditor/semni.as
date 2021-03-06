package
{
	import Box2D.Dynamics.*;
	import Box2D.Collision.*;
	import Box2D.Collision.Shapes.*;
	import Box2D.Common.Math.*;
    import flash.utils.Dictionary;

    public class PhysicsData extends Object
	{
		// ptm ratio
        public var ptm_ratio:Number = 1500;
		
		// the physcis data 
		var dict:Dictionary;
		
        //
        // bodytype:
        //  b2_staticBody
        //  b2_kinematicBody
        //  b2_dynamicBody

        public function createBody(name:String, world:b2World, bodyType:uint, userData:*):b2Body
        {
            var fixtures:Array = dict[name];

            var body:b2Body;
            var f:Number;

            // prepare body def
            var bodyDef:b2BodyDef = new b2BodyDef();
            bodyDef.type = bodyType;
            bodyDef.userData = userData;

            // create the body
            body = world.CreateBody(bodyDef);

            // prepare fixtures
            for(f=0; f<fixtures.length; f++)
            {
                var fixture:Array = fixtures[f];

                var fixtureDef:b2FixtureDef = new b2FixtureDef();

                fixtureDef.density=fixture[0];
                fixtureDef.friction=fixture[1];
                fixtureDef.restitution=fixture[2];

                fixtureDef.filter.categoryBits = fixture[3];
                fixtureDef.filter.maskBits = fixture[4];
                fixtureDef.filter.groupIndex = fixture[5];
                fixtureDef.isSensor = fixture[6];

                if(fixture[7] == "POLYGON")
                {                    
                    var p:Number;
                    var polygons:Array = fixture[8];
                    for(p=0; p<polygons.length; p++)
                    {
                        var polygonShape:b2PolygonShape = new b2PolygonShape();
                        polygonShape.SetAsArray(polygons[p], polygons[p].length);
                        fixtureDef.shape=polygonShape;

                        body.CreateFixture(fixtureDef);
                    }
                }
                else if(fixture[7] == "CIRCLE")
                {
                    var circleShape:b2CircleShape = new b2CircleShape(fixture[9]);                    
                    circleShape.SetLocalPosition(fixture[8]);
                    fixtureDef.shape=circleShape;
                    body.CreateFixture(fixtureDef);                    
                }                
            }

            return body;
        }

		
        public function PhysicsData(): void
		{
			dict = new Dictionary();
			

			dict["semni"] = [

										[
											// density, friction, restitution
                                            2, 0, 0,
                                            // categoryBits, maskBits, groupIndex, isSensor
											1, 65535, 0, false,
											'POLYGON',

                                            // vertexes of decomposed polygons
                                            [

                                                [   new b2Vec2(278/ptm_ratio, 485/ptm_ratio)  ,  new b2Vec2(294/ptm_ratio, 461/ptm_ratio)  ,  new b2Vec2(338/ptm_ratio, 459/ptm_ratio)  ,  new b2Vec2(382/ptm_ratio, 491/ptm_ratio)  ,  new b2Vec2(358/ptm_ratio, 580/ptm_ratio)  ,  new b2Vec2(319/ptm_ratio, 551/ptm_ratio)  ,  new b2Vec2(284/ptm_ratio, 521/ptm_ratio)  ,  new b2Vec2(276/ptm_ratio, 501/ptm_ratio)  ] ,
                                                [   new b2Vec2(294/ptm_ratio, 461/ptm_ratio)  ,  new b2Vec2(278/ptm_ratio, 485/ptm_ratio)  ,  new b2Vec2(283/ptm_ratio, 472/ptm_ratio)  ] ,
                                                [   new b2Vec2(543/ptm_ratio, 671/ptm_ratio)  ,  new b2Vec2(499/ptm_ratio, 655/ptm_ratio)  ,  new b2Vec2(517/ptm_ratio, 537/ptm_ratio)  ,  new b2Vec2(545/ptm_ratio, 539/ptm_ratio)  ,  new b2Vec2(584/ptm_ratio, 545/ptm_ratio)  ,  new b2Vec2(622/ptm_ratio, 608/ptm_ratio)  ,  new b2Vec2(590/ptm_ratio, 662/ptm_ratio)  ,  new b2Vec2(570/ptm_ratio, 671/ptm_ratio)  ] ,
                                                [   new b2Vec2(338/ptm_ratio, 459/ptm_ratio)  ,  new b2Vec2(294/ptm_ratio, 461/ptm_ratio)  ,  new b2Vec2(308/ptm_ratio, 455/ptm_ratio)  ,  new b2Vec2(322/ptm_ratio, 454/ptm_ratio)  ] ,
                                                [   new b2Vec2(446/ptm_ratio, 521/ptm_ratio)  ,  new b2Vec2(402/ptm_ratio, 608/ptm_ratio)  ,  new b2Vec2(358/ptm_ratio, 580/ptm_ratio)  ,  new b2Vec2(382/ptm_ratio, 491/ptm_ratio)  ] ,
                                                [   new b2Vec2(517/ptm_ratio, 537/ptm_ratio)  ,  new b2Vec2(499/ptm_ratio, 655/ptm_ratio)  ,  new b2Vec2(468/ptm_ratio, 642/ptm_ratio)  ,  new b2Vec2(402/ptm_ratio, 608/ptm_ratio)  ,  new b2Vec2(446/ptm_ratio, 521/ptm_ratio)  ] ,
                                                [   new b2Vec2(606/ptm_ratio, 649/ptm_ratio)  ,  new b2Vec2(590/ptm_ratio, 662/ptm_ratio)  ,  new b2Vec2(622/ptm_ratio, 608/ptm_ratio)  ,  new b2Vec2(618/ptm_ratio, 629/ptm_ratio)  ] ,
                                                [   new b2Vec2(613/ptm_ratio, 571/ptm_ratio)  ,  new b2Vec2(620/ptm_ratio, 589/ptm_ratio)  ,  new b2Vec2(622/ptm_ratio, 608/ptm_ratio)  ,  new b2Vec2(584/ptm_ratio, 545/ptm_ratio)  ,  new b2Vec2(601/ptm_ratio, 556/ptm_ratio)  ] ,
                                                [   new b2Vec2(584/ptm_ratio, 545/ptm_ratio)  ,  new b2Vec2(545/ptm_ratio, 539/ptm_ratio)  ,  new b2Vec2(565/ptm_ratio, 539/ptm_ratio)  ]
											]

										]
 ,
										[
											// density, friction, restitution
                                            2, 0, 0,
                                            // categoryBits, maskBits, groupIndex, isSensor
											1, 65535, 0, false,
											'POLYGON',

                                            // vertexes of decomposed polygons
                                            [

                                                [   new b2Vec2(885/ptm_ratio, 536/ptm_ratio)  ,  new b2Vec2(851/ptm_ratio, 583/ptm_ratio)  ,  new b2Vec2(814/ptm_ratio, 619/ptm_ratio)  ,  new b2Vec2(691/ptm_ratio, 473/ptm_ratio)  ,  new b2Vec2(804/ptm_ratio, 397/ptm_ratio)  ,  new b2Vec2(844/ptm_ratio, 410/ptm_ratio)  ,  new b2Vec2(878/ptm_ratio, 443/ptm_ratio)  ,  new b2Vec2(893/ptm_ratio, 488/ptm_ratio)  ] ,
                                                [   new b2Vec2(804/ptm_ratio, 397/ptm_ratio)  ,  new b2Vec2(691/ptm_ratio, 473/ptm_ratio)  ,  new b2Vec2(720/ptm_ratio, 429/ptm_ratio)  ,  new b2Vec2(760/ptm_ratio, 402/ptm_ratio)  ] ,
                                                [   new b2Vec2(814/ptm_ratio, 619/ptm_ratio)  ,  new b2Vec2(776/ptm_ratio, 644/ptm_ratio)  ,  new b2Vec2(655/ptm_ratio, 508/ptm_ratio)  ,  new b2Vec2(691/ptm_ratio, 473/ptm_ratio)  ] ,
                                                [   new b2Vec2(776/ptm_ratio, 644/ptm_ratio)  ,  new b2Vec2(731/ptm_ratio, 665/ptm_ratio)  ,  new b2Vec2(681/ptm_ratio, 679/ptm_ratio)  ,  new b2Vec2(628/ptm_ratio, 684/ptm_ratio)  ,  new b2Vec2(602/ptm_ratio, 534/ptm_ratio)  ,  new b2Vec2(655/ptm_ratio, 508/ptm_ratio)  ] ,
                                                [   new b2Vec2(628/ptm_ratio, 684/ptm_ratio)  ,  new b2Vec2(593/ptm_ratio, 681/ptm_ratio)  ,  new b2Vec2(559/ptm_ratio, 675/ptm_ratio)  ,  new b2Vec2(531/ptm_ratio, 667/ptm_ratio)  ,  new b2Vec2(510/ptm_ratio, 654/ptm_ratio)  ,  new b2Vec2(526/ptm_ratio, 546/ptm_ratio)  ,  new b2Vec2(548/ptm_ratio, 539/ptm_ratio)  ,  new b2Vec2(602/ptm_ratio, 534/ptm_ratio)  ] ,
                                                [   new b2Vec2(526/ptm_ratio, 546/ptm_ratio)  ,  new b2Vec2(510/ptm_ratio, 654/ptm_ratio)  ,  new b2Vec2(498/ptm_ratio, 639/ptm_ratio)  ,  new b2Vec2(490/ptm_ratio, 618/ptm_ratio)  ,  new b2Vec2(489/ptm_ratio, 598/ptm_ratio)  ,  new b2Vec2(496/ptm_ratio, 575/ptm_ratio)  ,  new b2Vec2(509/ptm_ratio, 557/ptm_ratio)  ]
											]

										]
 ,
										[
											// density, friction, restitution
                                            2, 0, 0,
                                            // categoryBits, maskBits, groupIndex, isSensor
											1, 65535, 0, false,
											'POLYGON',

                                            // vertexes of decomposed polygons
                                            [

                                                [   new b2Vec2(276/ptm_ratio, 501/ptm_ratio)  ,  new b2Vec2(277/ptm_ratio, 487/ptm_ratio)  ,  new b2Vec2(293/ptm_ratio, 461/ptm_ratio)  ,  new b2Vec2(334/ptm_ratio, 429/ptm_ratio)  ,  new b2Vec2(341/ptm_ratio, 568/ptm_ratio)  ,  new b2Vec2(293/ptm_ratio, 529/ptm_ratio)  ,  new b2Vec2(280/ptm_ratio, 515/ptm_ratio)  ] ,
                                                [   new b2Vec2(293/ptm_ratio, 461/ptm_ratio)  ,  new b2Vec2(277/ptm_ratio, 487/ptm_ratio)  ,  new b2Vec2(282/ptm_ratio, 474/ptm_ratio)  ] ,
                                                [   new b2Vec2(368/ptm_ratio, 405/ptm_ratio)  ,  new b2Vec2(411/ptm_ratio, 378/ptm_ratio)  ,  new b2Vec2(450/ptm_ratio, 358/ptm_ratio)  ,  new b2Vec2(420/ptm_ratio, 618/ptm_ratio)  ,  new b2Vec2(396/ptm_ratio, 605/ptm_ratio)  ,  new b2Vec2(341/ptm_ratio, 568/ptm_ratio)  ,  new b2Vec2(334/ptm_ratio, 429/ptm_ratio)  ] ,
                                                [   new b2Vec2(1146/ptm_ratio, 429/ptm_ratio)  ,  new b2Vec2(1162/ptm_ratio, 473/ptm_ratio)  ,  new b2Vec2(1164/ptm_ratio, 495/ptm_ratio)  ,  new b2Vec2(1025/ptm_ratio, 647/ptm_ratio)  ,  new b2Vec2(1059/ptm_ratio, 360/ptm_ratio)  ,  new b2Vec2(1086/ptm_ratio, 374/ptm_ratio)  ,  new b2Vec2(1111/ptm_ratio, 390/ptm_ratio)  ,  new b2Vec2(1130/ptm_ratio, 407/ptm_ratio)  ] ,
                                                [   new b2Vec2(1162/ptm_ratio, 473/ptm_ratio)  ,  new b2Vec2(1146/ptm_ratio, 429/ptm_ratio)  ,  new b2Vec2(1156/ptm_ratio, 450/ptm_ratio)  ] ,
                                                [   new b2Vec2(1118/ptm_ratio, 595/ptm_ratio)  ,  new b2Vec2(1096/ptm_ratio, 612/ptm_ratio)  ,  new b2Vec2(1060/ptm_ratio, 631/ptm_ratio)  ,  new b2Vec2(1025/ptm_ratio, 647/ptm_ratio)  ,  new b2Vec2(1164/ptm_ratio, 495/ptm_ratio)  ,  new b2Vec2(1156/ptm_ratio, 540/ptm_ratio)  ,  new b2Vec2(1147/ptm_ratio, 560/ptm_ratio)  ,  new b2Vec2(1136/ptm_ratio, 577/ptm_ratio)  ] ,
                                                [   new b2Vec2(1156/ptm_ratio, 540/ptm_ratio)  ,  new b2Vec2(1164/ptm_ratio, 495/ptm_ratio)  ,  new b2Vec2(1162/ptm_ratio, 518/ptm_ratio)  ] ,
                                                [   new b2Vec2(450/ptm_ratio, 358/ptm_ratio)  ,  new b2Vec2(468/ptm_ratio, 349/ptm_ratio)  ,  new b2Vec2(528/ptm_ratio, 666/ptm_ratio)  ,  new b2Vec2(506/ptm_ratio, 658/ptm_ratio)  ,  new b2Vec2(441/ptm_ratio, 629/ptm_ratio)  ,  new b2Vec2(420/ptm_ratio, 618/ptm_ratio)  ] ,
                                                [   new b2Vec2(1025/ptm_ratio, 647/ptm_ratio)  ,  new b2Vec2(998/ptm_ratio, 658/ptm_ratio)  ,  new b2Vec2(970/ptm_ratio, 324/ptm_ratio)  ,  new b2Vec2(994/ptm_ratio, 332/ptm_ratio)  ,  new b2Vec2(1026/ptm_ratio, 345/ptm_ratio)  ,  new b2Vec2(1059/ptm_ratio, 360/ptm_ratio)  ] ,
                                                [   new b2Vec2(861/ptm_ratio, 694/ptm_ratio)  ,  new b2Vec2(878/ptm_ratio, 691/ptm_ratio)  ,  new b2Vec2(870/ptm_ratio, 693/ptm_ratio)  ] ,
                                                [   new b2Vec2(503/ptm_ratio, 334/ptm_ratio)  ,  new b2Vec2(553/ptm_ratio, 317/ptm_ratio)  ,  new b2Vec2(546/ptm_ratio, 672/ptm_ratio)  ,  new b2Vec2(528/ptm_ratio, 666/ptm_ratio)  ,  new b2Vec2(468/ptm_ratio, 349/ptm_ratio)  ] ,
                                                [   new b2Vec2(998/ptm_ratio, 658/ptm_ratio)  ,  new b2Vec2(981/ptm_ratio, 664/ptm_ratio)  ,  new b2Vec2(965/ptm_ratio, 669/ptm_ratio)  ,  new b2Vec2(932/ptm_ratio, 312/ptm_ratio)  ,  new b2Vec2(950/ptm_ratio, 317/ptm_ratio)  ,  new b2Vec2(970/ptm_ratio, 324/ptm_ratio)  ] ,
                                                [   new b2Vec2(623/ptm_ratio, 300/ptm_ratio)  ,  new b2Vec2(546/ptm_ratio, 672/ptm_ratio)  ,  new b2Vec2(587/ptm_ratio, 308/ptm_ratio)  ,  new b2Vec2(605/ptm_ratio, 303/ptm_ratio)  ] ,
                                                [   new b2Vec2(786/ptm_ratio, 289/ptm_ratio)  ,  new b2Vec2(928/ptm_ratio, 680/ptm_ratio)  ,  new b2Vec2(731/ptm_ratio, 288/ptm_ratio)  ,  new b2Vec2(749/ptm_ratio, 287/ptm_ratio)  ] ,
                                                [   new b2Vec2(947/ptm_ratio, 675/ptm_ratio)  ,  new b2Vec2(928/ptm_ratio, 680/ptm_ratio)  ,  new b2Vec2(879/ptm_ratio, 300/ptm_ratio)  ,  new b2Vec2(897/ptm_ratio, 303/ptm_ratio)  ,  new b2Vec2(915/ptm_ratio, 307/ptm_ratio)  ,  new b2Vec2(932/ptm_ratio, 312/ptm_ratio)  ,  new b2Vec2(965/ptm_ratio, 669/ptm_ratio)  ] ,
                                                [   new b2Vec2(570/ptm_ratio, 312/ptm_ratio)  ,  new b2Vec2(587/ptm_ratio, 308/ptm_ratio)  ,  new b2Vec2(546/ptm_ratio, 672/ptm_ratio)  ,  new b2Vec2(553/ptm_ratio, 317/ptm_ratio)  ] ,
                                                [   new b2Vec2(928/ptm_ratio, 680/ptm_ratio)  ,  new b2Vec2(786/ptm_ratio, 289/ptm_ratio)  ,  new b2Vec2(843/ptm_ratio, 294/ptm_ratio)  ,  new b2Vec2(879/ptm_ratio, 300/ptm_ratio)  ] ,
                                                [   new b2Vec2(576/ptm_ratio, 680/ptm_ratio)  ,  new b2Vec2(639/ptm_ratio, 297/ptm_ratio)  ,  new b2Vec2(671/ptm_ratio, 698/ptm_ratio)  ,  new b2Vec2(640/ptm_ratio, 694/ptm_ratio)  ,  new b2Vec2(593/ptm_ratio, 685/ptm_ratio)  ] ,
                                                [   new b2Vec2(782/ptm_ratio, 702/ptm_ratio)  ,  new b2Vec2(639/ptm_ratio, 297/ptm_ratio)  ,  new b2Vec2(653/ptm_ratio, 295/ptm_ratio)  ,  new b2Vec2(807/ptm_ratio, 701/ptm_ratio)  ,  new b2Vec2(797/ptm_ratio, 702/ptm_ratio)  ] ,
                                                [   new b2Vec2(639/ptm_ratio, 297/ptm_ratio)  ,  new b2Vec2(576/ptm_ratio, 680/ptm_ratio)  ,  new b2Vec2(546/ptm_ratio, 672/ptm_ratio)  ,  new b2Vec2(623/ptm_ratio, 300/ptm_ratio)  ] ,
                                                [   new b2Vec2(697/ptm_ratio, 290/ptm_ratio)  ,  new b2Vec2(928/ptm_ratio, 680/ptm_ratio)  ,  new b2Vec2(888/ptm_ratio, 689/ptm_ratio)  ,  new b2Vec2(807/ptm_ratio, 701/ptm_ratio)  ,  new b2Vec2(666/ptm_ratio, 293/ptm_ratio)  ,  new b2Vec2(680/ptm_ratio, 291/ptm_ratio)  ] ,
                                                [   new b2Vec2(639/ptm_ratio, 297/ptm_ratio)  ,  new b2Vec2(704/ptm_ratio, 701/ptm_ratio)  ,  new b2Vec2(687/ptm_ratio, 700/ptm_ratio)  ,  new b2Vec2(671/ptm_ratio, 698/ptm_ratio)  ] ,
                                                [   new b2Vec2(639/ptm_ratio, 297/ptm_ratio)  ,  new b2Vec2(782/ptm_ratio, 702/ptm_ratio)  ,  new b2Vec2(767/ptm_ratio, 703/ptm_ratio)  ,  new b2Vec2(735/ptm_ratio, 703/ptm_ratio)  ,  new b2Vec2(704/ptm_ratio, 701/ptm_ratio)  ] ,
                                                [   new b2Vec2(928/ptm_ratio, 680/ptm_ratio)  ,  new b2Vec2(697/ptm_ratio, 290/ptm_ratio)  ,  new b2Vec2(731/ptm_ratio, 288/ptm_ratio)  ] ,
                                                [   new b2Vec2(666/ptm_ratio, 293/ptm_ratio)  ,  new b2Vec2(807/ptm_ratio, 701/ptm_ratio)  ,  new b2Vec2(653/ptm_ratio, 295/ptm_ratio)  ] ,
                                                [   new b2Vec2(907/ptm_ratio, 685/ptm_ratio)  ,  new b2Vec2(928/ptm_ratio, 680/ptm_ratio)  ,  new b2Vec2(916/ptm_ratio, 683/ptm_ratio)  ] ,
                                                [   new b2Vec2(888/ptm_ratio, 689/ptm_ratio)  ,  new b2Vec2(907/ptm_ratio, 685/ptm_ratio)  ,  new b2Vec2(898/ptm_ratio, 687/ptm_ratio)  ] ,
                                                [   new b2Vec2(854/ptm_ratio, 695/ptm_ratio)  ,  new b2Vec2(814/ptm_ratio, 700/ptm_ratio)  ,  new b2Vec2(888/ptm_ratio, 689/ptm_ratio)  ] ,
                                                [   new b2Vec2(832/ptm_ratio, 698/ptm_ratio)  ,  new b2Vec2(847/ptm_ratio, 696/ptm_ratio)  ,  new b2Vec2(840/ptm_ratio, 697/ptm_ratio)  ]
											]

										]
 ,
										[
											// density, friction, restitution
                                            2, 0, 0,
                                            // categoryBits, maskBits, groupIndex, isSensor
											1, 65535, 0, false,
											'POLYGON',

                                            // vertexes of decomposed polygons
                                            [

                                                [   new b2Vec2(554/ptm_ratio, 605/ptm_ratio)  ,  new b2Vec2(201/ptm_ratio, 764/ptm_ratio)  ,  new b2Vec2(203/ptm_ratio, 753/ptm_ratio)  ]
											]

										]
 ,
										[
											// density, friction, restitution
                                            2, 0, 0,
                                            // categoryBits, maskBits, groupIndex, isSensor
											1, 65535, 0, false,
											'POLYGON',

                                            // vertexes of decomposed polygons
                                            [

                                                [   new b2Vec2(794/ptm_ratio, 496/ptm_ratio)  ,  new b2Vec2(645/ptm_ratio, 282/ptm_ratio)  ,  new b2Vec2(659/ptm_ratio, 279/ptm_ratio)  ]
											]

										]
 ,
										[
											// density, friction, restitution
                                            2, 0, 0,
                                            // categoryBits, maskBits, groupIndex, isSensor
											1, 65535, 0, false,
											'CIRCLE',

                                            // center, radius
                                            new b2Vec2(1030.545/ptm_ratio,496.364/ptm_ratio),
                                            117.004/ptm_ratio

										]
 ,
										[
											// density, friction, restitution
                                            2, 0, 0,
                                            // categoryBits, maskBits, groupIndex, isSensor
											1, 65535, 0, false,
											'POLYGON',

                                            // vertexes of decomposed polygons
                                            [

                                                [   new b2Vec2(389/ptm_ratio, 632/ptm_ratio)  ,  new b2Vec2(378/ptm_ratio, 616/ptm_ratio)  ,  new b2Vec2(447/ptm_ratio, 568/ptm_ratio)  ]
											]

										]
 ,
										[
											// density, friction, restitution
                                            2, 0, 0,
                                            // categoryBits, maskBits, groupIndex, isSensor
											1, 65535, 0, false,
											'POLYGON',

                                            // vertexes of decomposed polygons
                                            [

                                                [   new b2Vec2(758/ptm_ratio, 752/ptm_ratio)  ,  new b2Vec2(742/ptm_ratio, 753/ptm_ratio)  ,  new b2Vec2(743/ptm_ratio, 535/ptm_ratio)  ]
											]

										]
 ,
										[
											// density, friction, restitution
                                            2, 0, 0,
                                            // categoryBits, maskBits, groupIndex, isSensor
											1, 65535, 0, false,
											'POLYGON',

                                            // vertexes of decomposed polygons
                                            [

                                                [   new b2Vec2(820/ptm_ratio, 502/ptm_ratio)  ,  new b2Vec2(842/ptm_ratio, 264/ptm_ratio)  ,  new b2Vec2(858/ptm_ratio, 266/ptm_ratio)  ]
											]

										]

									];

		}
	}
}
