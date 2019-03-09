package org.firstinspires.ftc.teamcode;
import org.junit.Test;
import static com.google.common.truth.Truth.assertThat;

public class PracticeTest {
    @Test
    public void addition_isCorrect() throws Exception {
        int result = 2 + 2;
        int expected = 4;
        assertThat(result).isEqualTo(expected);
    }
}
