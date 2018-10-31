paper.install(window);
window.onload = function(){
    paper.setup('canvas');

    var path;
    var stroke_render;
    var path_zs;


    var hw = 5;

    document.getElementById("canvas").addEventListener('pointerdown', function(event){
        if(path)
            path.selected = false;

        path = new Path({
            strokeColor: 'black',
            // fullySelected: true
        });
        path_zs = [];
        stroke_render = new CompoundPath({
            fillColor: 'grey',
        });
    }, false);

    document.getElementById("canvas").addEventListener('pointermove',  function(event){
        if(event.pressure>0){
            path.add(new Point(event.offsetX, event.offsetY));
            path_zs.push(event.pressure);
            
            var n = path.segments.length;

            var r = hw * path_zs[n-1];
            stroke_render.addChild(new Path.Ellipse({
                point: [event.offsetX - r, event.offsetY-r],
                size: [r*2, r*2],
            }));
            if(n>1){
                var curve = path.curves[n-2];
                var v = curve.point2.subtract(curve.point1).normalize()
                var q1 = new Point(-v.y,v.x).multiply(hw * path_zs[n-2]);
                var q2 = new Point(-v.y,v.x).multiply(hw * path_zs[n-1]);
                stroke_render.addChild(new Path({
                    segments: [
                        curve.point1.subtract(q1),
                        curve.point2.subtract(q2),
                        curve.point2.add     (q2),
                        curve.point1.add     (q1),
                    ],
                }));
            }

        }
    }, false);

    document.getElementById("canvas").addEventListener('pointerup',  function(event){
        var zpath = new Path({
            segment: path.segments.map(s=>[s.location.offset, path_zs[s.location.index]]),
            strokeColor: 'red',
            fullySelected: true
        })
        // zpath.position.x += 100
        // zpath.position.y += 100
        console.log(path_zs)

        var path2 = path.clone();
        path2.simplify(10);
        path2.flatten(.5);

        path.remove();
        stroke_render.remove();

        var l1 = path.length;
        var l2 = path2.length;
        var nos = path2.segments.map(s=>s.location.offset/l2);
        var is = nos.map(o=>path.getLocationAt(o*l1)).map(l=>l.index+(l.curveOffset)/l.curve.length);
        var path_zs2 = is.map(i=>path_zs[Math.floor(i)]*(i%1.)+path_zs[Math.ceil(i)]*(1-(i%1.)))


        var stroke_render2 = new CompoundPath({
            fillColor: 'grey',
        });

        for(var i=0; i<path2.segments.length; ++i){
            var r = path_zs2[i] * hw;
            var p = path2.segments[i].point;
            stroke_render2.addChild(new Path.Ellipse({
                point: [p.x-r, p.y-r],
                size: [r*2, r*2],
            }))
        }
        for(var i=0; i<path2.curves.length; ++i){
            var curve = path2.curves[i];
            var v = curve.point2.subtract(curve.point1).normalize()
            var q1 = new Point(-v.y,v.x).multiply(hw * path_zs2[i]);
            var q2 = new Point(-v.y,v.x).multiply(hw * path_zs2[i+1]);
            stroke_render2.addChild(new Path({
                segments: [
                    curve.point1.subtract(q1),
                    curve.point2.subtract(q2),
                    curve.point2.add     (q2),
                    curve.point1.add     (q1),
                ],
            }));
        }


        var xf = new Matrix();
        xf.scale(.2);

        var points = path2.segments.map(s=>xf.transform(s.point));
        var xyzs = [];
        for(var i=0; i<points.length; ++i)
            xyzs.push([points[i].x, points[i].y, path_zs2[i]]);
        console.log(xyzs)
        $.post("draw", JSON.stringify([xyzs]), function(rdata){
            console.log(rdata)
        });
    }, false);

}


function update_config_inputs(data){
    console.log(data);
        $('input#speed').val(data['speed_settings'][0]*100);
        $('input#accel').val(data['speed_settings'][1]*100);
        $('input#cornering').val(data['speed_settings'][2]*100);

        $('input#penpos1').val(data['pen_positions'][0]*100);
        $('input#penpos2').val(data['pen_positions'][1]*100);
        $('input#penpos3').val(data['pen_positions'][2]*100);
}

$(function(){
    console.log('hello');
    $.post('/get-config', update_config_inputs);

    $("input#speed, input#accel").change(function(){
        var config_update = {
            'speed_settings': [
                $("input#speed").val()/100., 
                $("input#accel").val()/100.,
                $("input#cornering").val()/100.,
            ]
        };
        $.post('update-config', JSON.stringify(config_update), update_config_inputs);
    });

    $("input.penpos").change(function(){
        $.post('move-pen', JSON.stringify($(this).val()/100.));

        var config_update = {
            'pen_positions': [
                $("input#penpos1").val()/100., 
                $("input#penpos2").val()/100.,
                $("input#penpos3").val()/100.
            ]
        };
        $.post('update-config', JSON.stringify(config_update), update_config_inputs);
    });

    // $("input#pendown").change(function(){
    //     $.post('move-pen', JSON.stringify($(this).val()/100.), function(data){}, 'json');
    // });
//     $("#slider1").slider({
//         range: true,
//         min: 0,
//         max: 100,
//         values: [25, 75],
//         stop: function(event, ui){
//             data = [
//                 ['pen-down-pos', ui.values[0]/100.],
//                 ['pen-up-pos'  , ui.values[1]/100.],
//                 [ui.handleIndex==1? 'raise-pen':'lower-pen'],
//             ]
//             $.post(".", JSON.stringify(data), function(rdata){
//                 console.log(data)
//             }, "application/json");
//             // if(ui.handleIndex == 1){
//             // }else{
//             //     $.post(".", JSON.stringify({'pen-down-pos': ui.value/100.}), function(rdata){
//             //         console.log(data)
//             //     }, "application/json");
//             // }
//             // $( "#amount" ).val( "$" + ui.values[ 0 ] + " - $" + ui.values[ 1 ] );
//         }
//     });
//     // $( "#amount" ).val( "$" + $( "#slider-range" ).slider( "values", 0 ) +" - $" + $( "#slider-range" ).slider( "values", 1 ) );
});