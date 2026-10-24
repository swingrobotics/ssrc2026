package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

/**
 * Guard against selecting a PathPlanner auto that references a missing path.
 *
 * This is a static deploy-asset check, not a simulation of robot motion.
 */
class PathPlannerAssetsTest {
  private static final Path DEPLOY = Path.of("src", "main", "deploy", "pathplanner");
  private static final Pattern PATH_REFERENCE =
      Pattern.compile("\\\"pathName\\\"\\s*:\\s*\\\"([^\\\"]+)\\\"");

  private static final Pattern NAMED_TYPE =
      Pattern.compile("\"type\"\\s*:\\s*\"named\"");
  private static final Pattern NAMED_REFERENCE =
      Pattern.compile("\"type\"\\s*:\\s*\"named\"\\s*,\\s*\"data\"\\s*:\\s*\\{\\s*\"name\"\\s*:\\s*(null|\"([^\"]*)\")");

  @Test
  void allAutoNamedCommandsAreValidAndRegistered() throws IOException {
    Path autosFolder = DEPLOY.resolve("autos");
    String robotContainer = Files.readString(
        Path.of("src", "main", "java", "frc", "robot", "RobotContainer.java"),
        StandardCharsets.UTF_8);
    try (Stream<Path> entries = Files.list(autosFolder)) {
      for (Path autoFile : entries.filter(p -> p.toString().endsWith(".auto")).toList()) {
        String json = Files.readString(autoFile, StandardCharsets.UTF_8);
        Matcher types = NAMED_TYPE.matcher(json);
        Matcher names = NAMED_REFERENCE.matcher(json);
        int typeCount = 0;
        int nameCount = 0;
        while (types.find()) {
          typeCount++;
        }
        while (names.find()) {
          nameCount++;
          String name = names.group(2);
          assertTrue(name != null && !name.isBlank(),
              () -> "Unnamed PathPlanner command in " + autoFile);
          assertTrue(robotContainer.contains(
              "NamedCommands.registerCommand(\"" + name + "\""),
              () -> "Unregistered PathPlanner command: " + name);
        }
        assertEquals(typeCount, nameCount,
            "Every named step must contain a valid command name in " + autoFile);
      }
    }
  }

  @Test
  void standaloneAutoFeederStopIsRegistered() throws IOException {
    String container = Files.readString(
        Path.of("src", "main", "java", "frc", "robot", "RobotContainer.java"),
        StandardCharsets.UTF_8);
    assertTrue(container.contains("NamedCommands.registerCommand(\"Auto Feeder Stop\""));
    assertTrue(container.contains("private void stopAutonomousFeeder()"));
  }

  @Test
  void testAutoStartsAndStopsIntakeThenShoots() throws IOException {
    String json = Files.readString(
        DEPLOY.resolve("autos").resolve("TestAuto.auto"), StandardCharsets.UTF_8);
    String[] steps = {
        "\"name\": \"Auto Hood Max\"",
        "\"pathName\": \"move 1\"",
        "\"name\": \"Auto Intake Start\"",
        "\"pathName\": \"intake 1\"",
        "\"pathName\": \"intake 2\"",
        "\"name\": \"Auto Intake Stop\"",
        "\"name\": \"Auto Shooter Start\"",
        "\"pathName\": \"move 2\"",
        "\"pathName\": \"shoot 1\"",
        "\"pathName\": \"shoot 2\"",
        "\"name\": \"Auto Shoot\"",
        "\"name\": \"Auto Shooter Stop\"",
        "\"name\": \"Auto Hood Down\""
    };
    int cursor = -1;
    for (String step : steps) {
      int next = json.indexOf(step, cursor + 1);
      assertTrue(next > cursor, "Missing or out-of-order auto step: " + step);
      cursor = next;
    }
  }

  @Test
  void selectableAutosReferenceExistingPathFiles() throws IOException {
    Path autosFolder = DEPLOY.resolve("autos");
    Path pathsFolder = DEPLOY.resolve("paths");
    assertTrue(Files.isDirectory(autosFolder), "Missing PathPlanner autos directory");
    assertTrue(Files.isDirectory(pathsFolder), "Missing PathPlanner paths directory");

    List<Path> autoFiles;
    try (Stream<Path> entries = Files.list(autosFolder)) {
      autoFiles = entries.filter(p -> p.getFileName().toString().endsWith(".auto"))
          .toList();
    }
    assertFalse(autoFiles.isEmpty(), "No deployable PathPlanner autos found");

    for (Path autoFile : autoFiles) {
      String json = Files.readString(autoFile, StandardCharsets.UTF_8);
      Matcher matches = PATH_REFERENCE.matcher(json);
      while (matches.find()) {
        String name = matches.group(1);
        Path pathFile = pathsFolder.resolve(name + ".path");
        assertTrue(Files.isRegularFile(pathFile),
            () -> "Auto " + autoFile.getFileName() + " references a missing path: "
                + pathFile.getFileName());
      }
    }
  }
}
