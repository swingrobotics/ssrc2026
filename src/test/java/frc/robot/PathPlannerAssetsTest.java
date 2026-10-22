package frc.robot;

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
