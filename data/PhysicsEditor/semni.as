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
        public var ptm_ratio:Number = 1000;
		
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
											'CIRCLE',

                                            // center, radius
                                            new b2Vec2(-10.000/ptm_ratio,722.000/ptm_ratio),
                                            10.000/ptm_ratio

										]
 ,
										[
											// density, friction, restitution
                                            2, 0, 0,
                                            // categoryBits, maskBits, groupIndex, isSensor
											1, 65535, 0, false,
											'CIRCLE',

                                            // center, radius
                                            new b2Vec2(-10.000/ptm_ratio,722.000/ptm_ratio),
                                            10.000/ptm_ratio

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

                                                [   new b2Vec2(153/ptm_ratio, 379/ptm_ratio)  ,  new b2Vec2(161/ptm_ratio, 368/ptm_ratio)  ,  new b2Vec2(178.66667175293/ptm_ratio, 358/ptm_ratio)  ,  new b2Vec2(230/ptm_ratio, 374/ptm_ratio)  ,  new b2Vec2(190/ptm_ratio, 447/ptm_ratio)  ,  new b2Vec2(158/ptm_ratio, 422/ptm_ratio)  ,  new b2Vec2(149/ptm_ratio, 400/ptm_ratio)  ] ,
                                                [   new b2Vec2(211/ptm_ratio, 359/ptm_ratio)  ,  new b2Vec2(230/ptm_ratio, 374/ptm_ratio)  ,  new b2Vec2(178.66667175293/ptm_ratio, 358/ptm_ratio)  ,  new b2Vec2(196/ptm_ratio, 355.333343505859/ptm_ratio)  ] ,
                                                [   new b2Vec2(487/ptm_ratio, 472/ptm_ratio)  ,  new b2Vec2(494/ptm_ratio, 490/ptm_ratio)  ,  new b2Vec2(496/ptm_ratio, 509/ptm_ratio)  ,  new b2Vec2(346/ptm_ratio, 539/ptm_ratio)  ,  new b2Vec2(278/ptm_ratio, 504/ptm_ratio)  ,  new b2Vec2(458/ptm_ratio, 447/ptm_ratio)  ,  new b2Vec2(475/ptm_ratio, 457/ptm_ratio)  ] ,
                                                [   new b2Vec2(399/ptm_ratio, 565/ptm_ratio)  ,  new b2Vec2(375/ptm_ratio, 554/ptm_ratio)  ,  new b2Vec2(346/ptm_ratio, 539/ptm_ratio)  ,  new b2Vec2(496/ptm_ratio, 509/ptm_ratio)  ,  new b2Vec2(491/ptm_ratio, 530/ptm_ratio)  ,  new b2Vec2(464/ptm_ratio, 563/ptm_ratio)  ,  new b2Vec2(445/ptm_ratio, 571/ptm_ratio)  ,  new b2Vec2(421/ptm_ratio, 572/ptm_ratio)  ] ,
                                                [   new b2Vec2(464/ptm_ratio, 563/ptm_ratio)  ,  new b2Vec2(491/ptm_ratio, 530/ptm_ratio)  ,  new b2Vec2(481/ptm_ratio, 548/ptm_ratio)  ] ,
                                                [   new b2Vec2(178.66667175293/ptm_ratio, 358/ptm_ratio)  ,  new b2Vec2(161/ptm_ratio, 368/ptm_ratio)  ,  new b2Vec2(170/ptm_ratio, 361/ptm_ratio)  ] ,
                                                [   new b2Vec2(230/ptm_ratio, 374/ptm_ratio)  ,  new b2Vec2(247/ptm_ratio, 386/ptm_ratio)  ,  new b2Vec2(233/ptm_ratio, 481/ptm_ratio)  ,  new b2Vec2(210/ptm_ratio, 464/ptm_ratio)  ,  new b2Vec2(190/ptm_ratio, 447/ptm_ratio)  ] ,
                                                [   new b2Vec2(268/ptm_ratio, 396/ptm_ratio)  ,  new b2Vec2(233/ptm_ratio, 481/ptm_ratio)  ,  new b2Vec2(247/ptm_ratio, 386/ptm_ratio)  ] ,
                                                [   new b2Vec2(332/ptm_ratio, 426/ptm_ratio)  ,  new b2Vec2(278/ptm_ratio, 504/ptm_ratio)  ,  new b2Vec2(253.33332824707/ptm_ratio, 492/ptm_ratio)  ,  new b2Vec2(233/ptm_ratio, 481/ptm_ratio)  ,  new b2Vec2(268/ptm_ratio, 396/ptm_ratio)  ] ,
                                                [   new b2Vec2(332/ptm_ratio, 426/ptm_ratio)  ,  new b2Vec2(458/ptm_ratio, 447/ptm_ratio)  ,  new b2Vec2(278/ptm_ratio, 504/ptm_ratio)  ] ,
                                                [   new b2Vec2(458/ptm_ratio, 447/ptm_ratio)  ,  new b2Vec2(418/ptm_ratio, 439/ptm_ratio)  ,  new b2Vec2(438/ptm_ratio, 440/ptm_ratio)  ] ,
                                                [   new b2Vec2(391/ptm_ratio, 435/ptm_ratio)  ,  new b2Vec2(332/ptm_ratio, 426/ptm_ratio)  ,  new b2Vec2(364/ptm_ratio, 430/ptm_ratio)  ] ,
                                                [   new b2Vec2(458/ptm_ratio, 447/ptm_ratio)  ,  new b2Vec2(391/ptm_ratio, 435/ptm_ratio)  ,  new b2Vec2(418/ptm_ratio, 439/ptm_ratio)  ]
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

                                                [   new b2Vec2(709/ptm_ratio, 306/ptm_ratio)  ,  new b2Vec2(747/ptm_ratio, 338/ptm_ratio)  ,  new b2Vec2(569/ptm_ratio, 577/ptm_ratio)  ,  new b2Vec2(523/ptm_ratio, 581/ptm_ratio)  ,  new b2Vec2(563/ptm_ratio, 374/ptm_ratio)  ,  new b2Vec2(583/ptm_ratio, 342/ptm_ratio)  ,  new b2Vec2(649/ptm_ratio, 299/ptm_ratio)  ,  new b2Vec2(681/ptm_ratio, 297/ptm_ratio)  ] ,
                                                [   new b2Vec2(415/ptm_ratio, 441/ptm_ratio)  ,  new b2Vec2(439/ptm_ratio, 438/ptm_ratio)  ,  new b2Vec2(374/ptm_ratio, 471/ptm_ratio)  ,  new b2Vec2(379/ptm_ratio, 463/ptm_ratio)  ,  new b2Vec2(398/ptm_ratio, 447/ptm_ratio)  ] ,
                                                [   new b2Vec2(767/ptm_ratio, 393/ptm_ratio)  ,  new b2Vec2(764/ptm_ratio, 419/ptm_ratio)  ,  new b2Vec2(754/ptm_ratio, 445/ptm_ratio)  ,  new b2Vec2(715/ptm_ratio, 495/ptm_ratio)  ,  new b2Vec2(649/ptm_ratio, 546/ptm_ratio)  ,  new b2Vec2(569/ptm_ratio, 577/ptm_ratio)  ,  new b2Vec2(747/ptm_ratio, 338/ptm_ratio)  ,  new b2Vec2(762/ptm_ratio, 365/ptm_ratio)  ] ,
                                                [   new b2Vec2(747/ptm_ratio, 338/ptm_ratio)  ,  new b2Vec2(709/ptm_ratio, 306/ptm_ratio)  ,  new b2Vec2(730/ptm_ratio, 319/ptm_ratio)  ] ,
                                                [   new b2Vec2(374/ptm_ratio, 544/ptm_ratio)  ,  new b2Vec2(366/ptm_ratio, 524/ptm_ratio)  ,  new b2Vec2(431/ptm_ratio, 573/ptm_ratio)  ,  new b2Vec2(413/ptm_ratio, 570/ptm_ratio)  ,  new b2Vec2(396/ptm_ratio, 564/ptm_ratio)  ,  new b2Vec2(384/ptm_ratio, 556/ptm_ratio)  ] ,
                                                [   new b2Vec2(366/ptm_ratio, 524/ptm_ratio)  ,  new b2Vec2(363/ptm_ratio, 509/ptm_ratio)  ,  new b2Vec2(364/ptm_ratio, 496/ptm_ratio)  ,  new b2Vec2(485/ptm_ratio, 430/ptm_ratio)  ,  new b2Vec2(527/ptm_ratio, 410/ptm_ratio)  ,  new b2Vec2(523/ptm_ratio, 581/ptm_ratio)  ,  new b2Vec2(477/ptm_ratio, 582/ptm_ratio)  ,  new b2Vec2(431/ptm_ratio, 573/ptm_ratio)  ] ,
                                                [   new b2Vec2(485/ptm_ratio, 430/ptm_ratio)  ,  new b2Vec2(364/ptm_ratio, 496/ptm_ratio)  ,  new b2Vec2(367/ptm_ratio, 485/ptm_ratio)  ,  new b2Vec2(374/ptm_ratio, 471/ptm_ratio)  ,  new b2Vec2(439/ptm_ratio, 438/ptm_ratio)  ] ,
                                                [   new b2Vec2(649/ptm_ratio, 299/ptm_ratio)  ,  new b2Vec2(583/ptm_ratio, 342/ptm_ratio)  ,  new b2Vec2(606/ptm_ratio, 318/ptm_ratio)  ,  new b2Vec2(624/ptm_ratio, 307/ptm_ratio)  ] ,
                                                [   new b2Vec2(563/ptm_ratio, 374/ptm_ratio)  ,  new b2Vec2(523/ptm_ratio, 581/ptm_ratio)  ,  new b2Vec2(527/ptm_ratio, 410/ptm_ratio)  ]
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

                                                [   new b2Vec2(150/ptm_ratio, 388/ptm_ratio)  ,  new b2Vec2(165/ptm_ratio, 364/ptm_ratio)  ,  new b2Vec2(208/ptm_ratio, 330/ptm_ratio)  ,  new b2Vec2(200/ptm_ratio, 457/ptm_ratio)  ,  new b2Vec2(173/ptm_ratio, 436/ptm_ratio)  ,  new b2Vec2(159/ptm_ratio, 423/ptm_ratio)  ,  new b2Vec2(150/ptm_ratio, 405/ptm_ratio)  ] ,
                                                [   new b2Vec2(165/ptm_ratio, 364/ptm_ratio)  ,  new b2Vec2(150/ptm_ratio, 388/ptm_ratio)  ,  new b2Vec2(155/ptm_ratio, 375/ptm_ratio)  ] ,
                                                [   new b2Vec2(208/ptm_ratio, 330/ptm_ratio)  ,  new b2Vec2(241/ptm_ratio, 306/ptm_ratio)  ,  new b2Vec2(284/ptm_ratio, 279/ptm_ratio)  ,  new b2Vec2(338/ptm_ratio, 251/ptm_ratio)  ,  new b2Vec2(407/ptm_ratio, 224/ptm_ratio)  ,  new b2Vec2(277/ptm_ratio, 510/ptm_ratio)  ,  new b2Vec2(234/ptm_ratio, 482/ptm_ratio)  ,  new b2Vec2(200/ptm_ratio, 457/ptm_ratio)  ] ,
                                                [   new b2Vec2(469/ptm_ratio, 207/ptm_ratio)  ,  new b2Vec2(527/ptm_ratio, 196/ptm_ratio)  ,  new b2Vec2(483/ptm_ratio, 589/ptm_ratio)  ,  new b2Vec2(426/ptm_ratio, 575/ptm_ratio)  ,  new b2Vec2(352/ptm_ratio, 548/ptm_ratio)  ,  new b2Vec2(314/ptm_ratio, 530/ptm_ratio)  ,  new b2Vec2(277/ptm_ratio, 510/ptm_ratio)  ,  new b2Vec2(407/ptm_ratio, 224/ptm_ratio)  ] ,
                                                [   new b2Vec2(1031/ptm_ratio, 354/ptm_ratio)  ,  new b2Vec2(1038/ptm_ratio, 396/ptm_ratio)  ,  new b2Vec2(1036/ptm_ratio, 419/ptm_ratio)  ,  new b2Vec2(1021/ptm_ratio, 463/ptm_ratio)  ,  new b2Vec2(963/ptm_ratio, 277/ptm_ratio)  ,  new b2Vec2(982/ptm_ratio, 288/ptm_ratio)  ,  new b2Vec2(1003/ptm_ratio, 306/ptm_ratio)  ,  new b2Vec2(1021/ptm_ratio, 330/ptm_ratio)  ] ,
                                                [   new b2Vec2(1038/ptm_ratio, 396/ptm_ratio)  ,  new b2Vec2(1031/ptm_ratio, 354/ptm_ratio)  ,  new b2Vec2(1037/ptm_ratio, 375/ptm_ratio)  ] ,
                                                [   new b2Vec2(582/ptm_ratio, 190/ptm_ratio)  ,  new b2Vec2(642/ptm_ratio, 189/ptm_ratio)  ,  new b2Vec2(706/ptm_ratio, 194/ptm_ratio)  ,  new b2Vec2(751/ptm_ratio, 200/ptm_ratio)  ,  new b2Vec2(589/ptm_ratio, 603/ptm_ratio)  ,  new b2Vec2(533/ptm_ratio, 598/ptm_ratio)  ,  new b2Vec2(483/ptm_ratio, 589/ptm_ratio)  ,  new b2Vec2(527/ptm_ratio, 196/ptm_ratio)  ] ,
                                                [   new b2Vec2(971/ptm_ratio, 512/ptm_ratio)  ,  new b2Vec2(933/ptm_ratio, 532/ptm_ratio)  ,  new b2Vec2(891/ptm_ratio, 551/ptm_ratio)  ,  new b2Vec2(811/ptm_ratio, 579/ptm_ratio)  ,  new b2Vec2(589/ptm_ratio, 603/ptm_ratio)  ,  new b2Vec2(1021/ptm_ratio, 463/ptm_ratio)  ,  new b2Vec2(1010/ptm_ratio, 478/ptm_ratio)  ,  new b2Vec2(995/ptm_ratio, 495/ptm_ratio)  ] ,
                                                [   new b2Vec2(794/ptm_ratio, 210/ptm_ratio)  ,  new b2Vec2(840/ptm_ratio, 223/ptm_ratio)  ,  new b2Vec2(891/ptm_ratio, 242/ptm_ratio)  ,  new b2Vec2(929/ptm_ratio, 259/ptm_ratio)  ,  new b2Vec2(1021/ptm_ratio, 463/ptm_ratio)  ,  new b2Vec2(589/ptm_ratio, 603/ptm_ratio)  ,  new b2Vec2(751/ptm_ratio, 200/ptm_ratio)  ] ,
                                                [   new b2Vec2(1021/ptm_ratio, 463/ptm_ratio)  ,  new b2Vec2(1036/ptm_ratio, 419/ptm_ratio)  ,  new b2Vec2(1031/ptm_ratio, 441/ptm_ratio)  ] ,
                                                [   new b2Vec2(1021/ptm_ratio, 463/ptm_ratio)  ,  new b2Vec2(929/ptm_ratio, 259/ptm_ratio)  ,  new b2Vec2(963/ptm_ratio, 277/ptm_ratio)  ] ,
                                                [   new b2Vec2(811/ptm_ratio, 579/ptm_ratio)  ,  new b2Vec2(891/ptm_ratio, 551/ptm_ratio)  ,  new b2Vec2(853/ptm_ratio, 566/ptm_ratio)  ] ,
                                                [   new b2Vec2(702/ptm_ratio, 600/ptm_ratio)  ,  new b2Vec2(665/ptm_ratio, 603/ptm_ratio)  ,  new b2Vec2(626/ptm_ratio, 604/ptm_ratio)  ,  new b2Vec2(589/ptm_ratio, 603/ptm_ratio)  ,  new b2Vec2(811/ptm_ratio, 579/ptm_ratio)  ,  new b2Vec2(774/ptm_ratio, 588/ptm_ratio)  ,  new b2Vec2(739/ptm_ratio, 595/ptm_ratio)  ]
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

                                                [   new b2Vec2(430/ptm_ratio, 504/ptm_ratio)  ,  new b2Vec2(201/ptm_ratio, 576/ptm_ratio)  ,  new b2Vec2(203/ptm_ratio, 565/ptm_ratio)  ]
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

                                                [   new b2Vec2(672/ptm_ratio, 385/ptm_ratio)  ,  new b2Vec2(698/ptm_ratio, 147/ptm_ratio)  ,  new b2Vec2(713/ptm_ratio, 148/ptm_ratio)  ]
											]

										]

									];

		}
	}
}
