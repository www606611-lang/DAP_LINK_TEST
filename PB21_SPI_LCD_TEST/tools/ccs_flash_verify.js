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
        session.expression.evaluate("g_pb21_pressed"));
    System.out.println("PB21_PRESS_COUNT=" +
        session.expression.evaluate("g_pb21_press_count"));
    System.out.println("PB21_RELEASE_COUNT=" +
        session.expression.evaluate("g_pb21_release_count"));
    System.out.println("PB21_LAST_CHANGE_MS=" +
        session.expression.evaluate("g_pb21_last_change_ms"));

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
