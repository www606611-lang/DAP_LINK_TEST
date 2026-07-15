importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);

if (arguments.length < 2) {
    System.err.println("Usage: dss ccs_flash_verify.js <target.ccxml> <program.out>");
    quit(2);
}

var environment = ScriptingEnvironment.instance();
var server = environment.getServer("DebugServer.1");
var session = null;

try {
    server.setConfig(arguments[0]);
    session = server.openSession("*", "*");
    session.target.connect();
    session.memory.loadProgram(arguments[1]);
    session.target.runAsynch();

    Thread.sleep(2000);
    session.target.halt();

    System.out.println("PB21_PRESSED=" +
        session.expression.evaluate("g_car_pb21_pressed"));
    System.out.println("PB21_PRESS_COUNT=" +
        session.expression.evaluate("g_car_pb21_press_count"));
    System.out.println("RESET_CAUSE=" +
        session.expression.evaluate("g_car_reset_cause"));
    System.out.println("CONTROL_MODE=" +
        session.expression.evaluate("g_car_control_mode"));
    System.out.println("CONTROL_BLOCK_REASON=" +
        session.expression.evaluate("g_car_control_block_reason"));
    System.out.println("MOTOR_HIGH_IMPEDANCE=" +
        session.expression.evaluate("g_car_motor_high_impedance"));
    System.out.println("ENCODER_SHADOW_ACTIVE=" +
        session.expression.evaluate("g_car_encoder_shadow_active"));
    System.out.println("ENCODER_0_COUNT=" +
        session.expression.evaluate("g_car_encoder_0_count"));
    System.out.println("ENCODER_0_SPEED_PPS=" +
        session.expression.evaluate("g_car_encoder_0_speed_pps"));
    System.out.println("ENCODER_0_INVALID=" +
        session.expression.evaluate("g_car_encoder_0_invalid"));
    System.out.println("ENCODER_1_COUNT=" +
        session.expression.evaluate("g_car_encoder_1_count"));
    System.out.println("ENCODER_1_SPEED_PPS=" +
        session.expression.evaluate("g_car_encoder_1_speed_pps"));
    System.out.println("ENCODER_1_INVALID=" +
        session.expression.evaluate("g_car_encoder_1_invalid"));

    session.target.runAsynch();
    session.target.disconnect();
    session.terminate();
    session = null;
    server.stop();
    quit(0);
} catch (error) {
    System.err.println("CCS flash/verify failed: " + error);

    if (session !== null) {
        try {
            session.target.disconnect();
        } catch (ignored) {
        }
        session.terminate();
    }

    server.stop();
    quit(1);
}
